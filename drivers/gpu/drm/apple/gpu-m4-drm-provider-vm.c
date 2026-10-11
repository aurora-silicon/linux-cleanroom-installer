// SPDX-License-Identifier: GPL-2.0-only
/* Private retained VM integration; no render registration or hardware writes. */
#include <linux/dma-resv.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <asm/cacheflush.h>
#include <drm/drm_gem.h>

#include "gpu-m4-drm-provider-vm.h"
#include "gpu-m4-drm-vm-tree.h"

struct gpu_m4_drm_provider {
	struct mutex internal_lock;
	struct mutex *lock;
	refcount_t refs;
	struct gpu_m4_drm_provider_config config;
	struct list_head vms, garbage_versions, garbage_objects, garbage_vms;
	struct list_head garbage_private;
	/* S3c: objects whose last reference an authority tree or an unlocked
	 * VM_BIND dropped (dead_lock); tree_work takes the provider mutex for
	 * their final puts and for deferred GEM/VM-close removals.
	 */
	spinlock_t dead_lock;
	struct list_head dead_objects;
	struct work_struct tree_work;
	/* Pointer-keyed membership of exactly the live vm->objects/vm->versions
	 * entries. A lookup compares bucket members' addresses only, so a
	 * caller-supplied identity is never dereferenced before membership.
	 */
	struct gpu_m4_drm_provider_object *member_object[256];
	struct gpu_m4_drm_provider_version *member_version[256];
	/* The same for exactly the VMs on p->vms: an O(1) membership proof in
	 * place of a walk over every retained (closed but referenced) VM.
	 */
	struct gpu_m4_drm_provider_vm *member_vm[256];
	struct list_head garbage_graphs, retained_graphs;
	/* S3c: VMs whose authority tree refused destruction, each with its own
	 * owning reference, context, holds, frontend, charge and provider
	 * reference: kept for good (the tree's leaves and cookie name them).
	 */
	struct list_head retained_vms;
	/* Per-ASID binding, index = TTBAT user context 1..63 ([0] is the
	 * firmware's context 0, never bound here): the resident graph, an
	 * installation in progress, and the wide binding generation. The
	 * generation advances only on a completed change of the installed root
	 * and is never reset, so an (ASID, generation) pair never repeats.
	 */
	struct gpu_m4_drm_provider_binding {
		struct gpu_m4_drm_provider_version *resident, *pending;
		u64 generation;
	} binding[GPU_M4_DRM_PROVIDER_ASIDS];
	u64 bytes;
	/* Removal-only replacement allowance (see charge): the footprint of the
	 * owner being replaced, live only inside one locked replacement section,
	 * and usable only within the provider-wide config.removal_reserve.
	 */
	u64 replacing;
	/* A charge was refused since the last reap: the reaper first drops every
	 * VM's optional construction cache, then refunds garbage as usual.
	 */
	bool pressure;
	/* Never-reused identity of each allocated version (same-user memo key). */
	u64 version_serial;
	/* Fault revision. Only a provider fault advances it, forcing the next
	 * ordinary refresh of every version to rescan its complete manifest.
	 * Binding begin/complete and unbind change which immutable graph is
	 * selected, never any graph's contents, so they do not advance it.
	 */
	u64 fault_revision;
	/* Last baseline build's distinct-reference stamp (see baseline_next). */
	u64 baseline_stamp;
	/* REMOVE-DEBUG: last holder census stamp, and retired composed images
	 * released (see version_release_image): versions, member objects whose
	 * hold they dropped, and releases skipped for lack of host memory.
	 */
	u64 census_stamp, images_released, image_objects, image_failures;
	int error;
};

struct gpu_m4_drm_provider_context {
	struct gpu_m4_drm_provider_vm *vm;
	refcount_t refs;
	struct page *page[4];
	void *nc;
};

#define GPU_M4_DRM_PROVIDER_OBJECT_HASH_BITS 7

struct gpu_m4_drm_provider_object;

struct gpu_m4_drm_provider_vm {
	struct list_head link, garbage, objects, versions;
	struct gpu_m4_drm_provider_vm *member_next; /* p->member_vm while on p->vms */
	struct list_head private_leases;
	struct gpu_m4_drm_provider *provider;
	struct gpu_m4_drm_vm *frontend;
	struct gpu_m4_drm_provider_context context;
	struct gpu_m4_drm_provider_version *active;
	/* Optional construction cache: callback-free image (nodes + manifest
	 * block) of the newest materialized user-base GRAPH, i.e. exactly the
	 * VM's common private baseline and no child's native extension. It owns
	 * its own charge, is never bindable or authority, is replaced/cleared
	 * through the reaper and is dropped at close (no VM->version cycle).
	 */
	struct gpu_m4_drm_provider_graph_garbage *graph_cache;
	u64 graph_cache_generation, graph_cache_revision;
	/* S3c span proofs of TREE versions: a span key proved by a complete
	 * live-tree walk at tree revision r stays proved while the tree's
	 * revision is still r (no leaf changed). Direct-mapped, lossy; provider
	 * mutex. tree_memo_pending is the revision read at the latest miss,
	 * BEFORE the walk its remember records.
	 */
	struct gpu_m4_provider_tree_memo *tree_memo;
	u64 tree_memo_pending;
	/* A second image under the same rules: the newest materialization whose
	 * key is not newer than the cache's (the demoted cache, or an older
	 * accepted capture's or the plain user-base graph), so those targets
	 * share it instead of each rebasing again from the newer cache.
	 */
	struct gpu_m4_drm_provider_graph_garbage *graph_spare;
	u64 graph_spare_generation, graph_spare_revision;
	/* Current immutable private native baseline (mapping-only membership of
	 * completed native units) and its private membership revision, distinct
	 * from the user ledger generation and the hardware binding generation.
	 * NULL/0 before the first member. Dropped (undiscoverable) at close;
	 * versions composed from a baseline keep their own reference.
	 */
	struct gpu_m4_drm_provider_baseline *baseline;
	u64 baseline_revision;
	/* Lockless capture ticket: the exact active version whose snapshot a
	 * locked capture already built (and its checked GRAPH, if any). Set
	 * under the provider mutex AND capture_lock; withdrawn under both before
	 * any active change, graph installation of that version, VM close and
	 * provider fault. Borrowed: the active owner's reference keeps it alive
	 * while published; a lockless capture takes its own with inc_not_zero.
	 */
	spinlock_t capture_lock;
	struct gpu_m4_drm_provider_version *capture;
	struct gpu_m4_drm_uat *capture_uat;
	/* Live (nonzero-ref) members of objects, indexed by GEM identity. */
	struct gpu_m4_drm_provider_object *object_hash[1U << GPU_M4_DRM_PROVIDER_OBJECT_HASH_BITS];
	refcount_t refs;
	u64 kernel_start, kernel_end, generation;
	u32 private_lease_count;
	/* VM-local fail-stop: a removal this VM's void BO close could not commit
	 * for lack of host memory. Its owners stay retained exactly; the VM, not
	 * the provider, accepts no further update or capture.
	 */
	int error;
	bool closed;
	/* REMOVE-DEBUG until S3c: shadow persistent tree mirroring the user
	 * leaves of active (provider mutex), never installed. NULL when off.
	 * With authority it is the VM's installed tree instead (see below).
	 */
	struct gpu_m4_vm_tree *tree;
	/*
	 * S3c (vm_tree_authority, sampled at create, ordinary profile only):
	 * the tree is the hardware authority and lives until the VM's final
	 * free. VM_BIND and GEM close edit only it (no version); jobs hold
	 * tree_version, the VM's one TREE version (O(1) job and binding
	 * handle, published under object_lock, dropped at close), and a tree
	 * guard. tree_holds are the private mappings' references the globals'
	 * DRIVER extents rely on, put after the tree is destroyed. The context
	 * is the VM's own and outlives the tree. object_lock orders
	 * object_hash against the unlocked VM_BIND lookup.
	 */
	bool authority;
	spinlock_t object_lock;
	struct gpu_m4_drm_provider_version *tree_version;
	struct gpu_m4_drm_uat_private *tree_holds;
	u32 tree_hold_count;
};

struct gpu_m4_drm_provider_private_lease {
	struct list_head link, garbage;
	struct gpu_m4_drm_provider_vm *vm;
	refcount_t refs;
	u64 va, size, start, end;
	/* Scratch of one locked baseline build: already listed in it. */
	u64 baseline_stamp;
};

/* Page-legitimacy admission of ONE retained native pin: every PA of exactly
 * this pin/PA vector passed config.validate_page once under the provider lock.
 * Holding the pin prevents migration/replacement for the admitted lifetime. A
 * new pin, changed inventory, CPU policy or session is a new admission. This
 * caches page legitimacy only, never job protection, current VA mappings,
 * reservations, visibility, completion or slot replacement.
 */
struct gpu_m4_drm_provider_admission {
	const void *session;
	const void *identity;
	struct gpu_m4_drm_bo_pin *pin;
	const u64 *pa;
	u64 size;
	enum gpu_m4_drm_cpu_policy cpu_policy;
	bool admitted;
};

struct gpu_m4_drm_provider_object {
	struct list_head link, garbage;
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_bo_pin *pin;
	struct gpu_m4_drm_vm_backing backing;
	struct gpu_m4_drm_provider_admission admission;
	struct gpu_m4_drm_provider_object *hash_next, *member_next;
	refcount_t refs;
	u64 charge;
	/* Scratch of one locked baseline build: already listed in it. */
	u64 baseline_stamp;
	/* REMOVE-DEBUG scratch of one locked holder census: its class there. */
	u64 census;
	u32 census_holder;
	bool hashed;
	/* S3c: queued on provider->dead_objects holding its last reference. */
	struct list_head dead;
};

struct gpu_m4_drm_provider_version {
	struct list_head link, garbage;
	struct gpu_m4_drm_provider_version *member_next;
	bool membered;
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_vm_ledger ledger;
	/* Retained validated VA/PA/identity index of exactly this immutable
	 * ledger (count-paired), built once at commit and reused by every later
	 * plan, logical proof, snapshot and BO close. Empty means "unproven".
	 */
	struct gpu_m4_drm_vm_ledger_index index;
	/* Non-NULL: ledger.leaf and index are borrowed from this held version's
	 * own immutable ones (a composed child shares its source's user map),
	 * never copied, written or freed here.
	 */
	struct gpu_m4_drm_provider_version *ledger_source;
	/* Expanded private manifest last proven against this whole user ledger.
	 * The next logical version with an identical manifest checks only its
	 * changed user leaves against it.
	 */
	struct gpu_m4_drm_vm_leaf *manifest;
	/* Its proven VA/PA/identity positions (charged with it). */
	struct gpu_m4_drm_vm_ledger_index manifest_index;
	u32 manifest_count;
	bool manifest_checked;
	struct gpu_m4_drm_vm_owner *snapshot;
	struct gpu_m4_drm_vm_owner *lineage;
	struct gpu_m4_drm_uat *uat;
	struct gpu_m4_drm_provider_object **objects;
	bool *object_used;
	struct gpu_m4_drm_uat_private *private;
	struct gpu_m4_drm_cache_access *native_private;
	struct gpu_m4_drm_provider_private_lease **private_leases;
	u32 object_count, private_count;
	u32 native_private_count, private_lease_count;
	/* objects[0, objects_sorted) were copied in identity order from an
	 * indexed inventory: indexing sorts only the appended tail and merges.
	 */
	u32 objects_sorted;
	refcount_t refs;
	u64 generation, table_bytes, metadata_bytes;
	/* fault_revision at which this GRAPH's proof stands: its construction
	 * proof (set at installation) or a later successful complete scan. 0
	 * after any failed check/scan: the next ordinary refresh rescans.
	 */
	u64 refreshed_revision;
	u32 private_held;
	/* private[0, base_private_count) is the VM's common private baseline
	 * (context + initialized globals); a composed version appends exactly
	 * its held native baseline's members after it, mapping-only (never in
	 * native_private[] or any job access set).
	 */
	u32 base_private_count;
	struct gpu_m4_drm_provider_baseline *baseline;
	/* Native baseline revision this version's user plan/commit carried. */
	u64 baseline_revision;
	/* Proof memos of facts of this immutable version once constructed: the
	 * leaf count of its complete private/claim shape check (0 = unproven),
	 * and the serial of a version whose user ledger was completely compared
	 * equal to this one. Serials are never reused, so no address can alias.
	 */
	u64 shape_leaves, serial, user_serial;
	/* RETIRED: a finished composed GRAPH that left residency and released
	 * its image and every baseline-member hold (version_release_image). Its
	 * snapshot, borrowed user ledger and source inventory remain, for the
	 * retained receipt/origin checks only: never bound, rebuilt or composed.
	 */
	enum {
		GPU_M4_PROVIDER_VERSION_UNREADY,
		GPU_M4_PROVIDER_VERSION_LOGICAL,
		GPU_M4_PROVIDER_VERSION_SNAPSHOT,
		GPU_M4_PROVIDER_VERSION_GRAPH,
		GPU_M4_PROVIDER_VERSION_RETIRED,
		/* S3c: the authority VM's one live handle: its persistent tree
		 * answers page/range/walk/roots; an empty snapshot (no user
		 * leaves or accesses) keeps the job/receipt identity checks.
		 * Never captured, composed, rebuilt or retired.
		 */
		GPU_M4_PROVIDER_VERSION_TREE,
	} state;
	bool objects_indexed;
	bool protected, working;
	/* Nonzero exactly while binding[binding_asid] names this version as its
	 * resident or pending graph. A version is bound in at most one ASID.
	 */
	u32 binding_asid;
	/* Composed only: its one job's work actually retired (version_retire).
	 * Every child composes its own version, so no new work ever starts on
	 * it; never cleared.
	 */
	bool finished;
	/* TREE: published jobs between begin_work and retire (working = !!). */
	u32 tree_work;
};

/* Only failed table construction uses this small deferred owner. Its exact
 * version reference keeps every native pin/charge and private cookie alive
 * until the partial UAT's callbacks can be released outside all locks.
 */
struct gpu_m4_drm_provider_graph_garbage {
	struct list_head link;
	struct gpu_m4_drm_provider_version *version;
	struct gpu_m4_drm_uat *uat;
	u64 bytes;
	/* Charge owners of the shared base pages this (failed) graph still
	 * names: the same-lineage append source version, or the VM's cache
	 * carrier. Released only after the partial UAT is released.
	 */
	struct gpu_m4_drm_provider_version *base_version;
	struct gpu_m4_drm_provider_graph_garbage *base_cache;
	/* Cache carrier only (lock-protected): constructions/failed graphs
	 * still borrowing its charged pages, and whether it already left the
	 * VM. Its refund waits for the last borrower.
	 */
	u32 borrows;
	bool detached;
};

/*
 * One typed mapping-only member of a native baseline: the actual native
 * object/pin (immutable PA inventory and policy), its typed private claim,
 * and the mapped VA/offset/span/access. The baseline holds one reference on
 * each distinct object and claim; never units, stages, histories or jobs.
 */
struct gpu_m4_drm_provider_baseline_member {
	struct gpu_m4_drm_provider_object *object;
	struct gpu_m4_drm_provider_private_lease *lease;
	u64 va, offset, size;
	/* Revision that admitted this exact member; every later baseline that
	 * still lists it carried it over (held throughout), never re-admitted.
	 */
	u64 since;
	u32 access;
};

/* Immutable, refcounted (under the provider lock), VA-ordered members;
 * objects are the distinct member objects in identity order. A commit
 * creates the next baseline; holders of an older one keep exactly it.
 */
struct gpu_m4_drm_provider_baseline {
	refcount_t refs;
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_provider_baseline_member *member;
	struct gpu_m4_drm_provider_object **objects;
	struct gpu_m4_drm_provider_private_lease **leases;
	u64 revision, charge, pages;
	u32 count, object_count, lease_count;
};

static void gpu_m4_drm_provider_baseline_put_locked(struct gpu_m4_drm_provider_baseline *b);
static void gpu_m4_drm_provider_tree_work(struct work_struct *work);
static bool gpu_m4_drm_provider_version_tree(const struct gpu_m4_drm_provider_version *v);
static int gpu_m4_drm_provider_tree_authority_private_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_uat_private *private, u32 count);
/* REMOVE-DEBUG until S3c: shadow-tree mirroring hooks (see the tree section). */
static void gpu_m4_drm_provider_tree_private_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_uat_private *private, u32 count);
static void gpu_m4_drm_provider_tree_composed_locked(struct gpu_m4_drm_provider_version *v);

/* A construction's borrow on the charge owner of its shared base pages. */
struct gpu_m4_drm_provider_graph_borrow {
	struct gpu_m4_drm_provider_version *version;
	struct gpu_m4_drm_provider_graph_garbage *cache;
};

/* REMOVE-DEBUG: read-only attribution of the shared charge at refusal, so a
 * capacity failure distinguishes live pins, retained versions and unreaped
 * garbage. Counts only; no reference or state changes. Provider lock held.
 */
static void gpu_m4_drm_provider_capacity_report(struct gpu_m4_drm_provider *p)
{
	struct gpu_m4_drm_provider_object *o;
	struct gpu_m4_drm_provider_version *v;
	struct gpu_m4_drm_provider_vm *vm;
	struct {
		u64 object_bytes, version_bytes, active_bytes, pinned_bytes, cache_bytes;
		u64 garbage_object_bytes, garbage_version_bytes;
		u32 vms, objects, versions, working;
		u32 garbage_objects, garbage_versions, garbage_vms;
	} c = { 0 };

	lockdep_assert_held(p->lock);
	list_for_each_entry(vm, &p->vms, link) {
		c.vms++;
		list_for_each_entry(o, &vm->objects, link) {
			c.objects++;
			c.object_bytes += o->charge;
			/* Observational only: per-VM pinned extents, never charged. */
			c.pinned_bytes += o->backing.size;
		}
		if (vm->graph_cache)
			c.cache_bytes += vm->graph_cache->bytes;
		if (vm->graph_spare)
			c.cache_bytes += vm->graph_spare->bytes;
		list_for_each_entry(v, &vm->versions, link) {
			c.versions++;
			c.version_bytes += v->metadata_bytes + v->table_bytes;
			c.working += v->protected || v->working;
		}
		if (vm->active)
			c.active_bytes += vm->active->metadata_bytes + vm->active->table_bytes;
	}
	list_for_each_entry(o, &p->garbage_objects, garbage) {
		c.garbage_objects++;
		c.garbage_object_bytes += o->charge;
	}
	list_for_each_entry(v, &p->garbage_versions, garbage) {
		c.garbage_versions++;
		c.garbage_version_bytes += v->metadata_bytes + v->table_bytes;
	}
	list_for_each_entry(vm, &p->garbage_vms, garbage)
		c.garbage_vms++;
	pr_warn("GPU provider capacity detail: vms=%u objects=%u object_bytes=%llu versions=%u version_bytes=%llu active_bytes=%llu resident_or_working=%u garbage_objects=%u garbage_object_bytes=%llu garbage_versions=%u garbage_version_bytes=%llu garbage_vms=%u pinned_bytes=%llu cache_bytes=%llu\n",
		c.vms, c.objects, c.object_bytes, c.versions, c.version_bytes, c.active_bytes,
		c.working, c.garbage_objects, c.garbage_object_bytes, c.garbage_versions,
		c.garbage_version_bytes, c.garbage_vms, c.pinned_bytes, c.cache_bytes);
}

/* A replacement may transiently leave bytes above the limit (see charge). */
static u64 gpu_m4_drm_provider_headroom(const struct gpu_m4_drm_provider *p)
{
	return p->bytes < p->config.memory_limit ? p->config.memory_limit - p->bytes : 0;
}

/* What a removal-only replacement may still charge: the provider-wide reserve
 * above the limit minus the excess ALREADY outstanding (bytes above the
 * limit). A replaced owner that stays held (captured job, residency, history)
 * keeps its charge, so successive replacements can never finance each other
 * beyond the reserve; consumed reserve returns only with actual refunds.
 */
static u64 gpu_m4_drm_provider_reserve_headroom(const struct gpu_m4_drm_provider *p)
{
	u64 ceiling = p->config.memory_limit + p->config.removal_reserve;

	return p->bytes < ceiling ? ceiling - p->bytes : 0;
}

/*
 * Every refusal is the caller's local -ENOSPC: it never latches provider
 * health. Inside a removal-only replacement section (p->replacing set by
 * the caller under this lock) a charge that does not fit is still admitted
 * while it stays within the replaced owner's own remaining footprint AND
 * within the provider-wide removal reserve (see reserve_headroom): the
 * replaced owner is refunded by the reaper once its last holder drops it,
 * but until then the excess is outstanding and bounded. Every charge in the
 * section consumes the section's allowance, so it never exceeds it in total.
 */
static int gpu_m4_drm_provider_charge(struct gpu_m4_drm_provider *p, u64 bytes)
{
	bool replacement = bytes <= p->replacing &&
		bytes <= gpu_m4_drm_provider_reserve_headroom(p);

	lockdep_assert_held(p->lock);
	p->replacing -= min(bytes, p->replacing);
	if (bytes > gpu_m4_drm_provider_headroom(p) && !replacement) {
		/* REMOVE-DEBUG: the budget covers provider-owned metadata only. */
		pr_warn("GPU provider capacity: used_bytes=%llu requested_bytes=%llu limit=%llu\n",
			p->bytes, bytes, p->config.memory_limit);
		gpu_m4_drm_provider_capacity_report(p);
		p->pressure = true;
		return -ENOSPC;
	}
	p->bytes += bytes;
	return 0;
}

struct gpu_m4_drm_provider *gpu_m4_drm_provider_create(
	const struct gpu_m4_drm_provider_config *config)
{
	struct gpu_m4_drm_provider *p;
	int ret;

	if (!config || !config->session || !config->get || !config->put ||
	    !config->validate_page || config->vm_start < GPU_M4_DRM_PAGE_SIZE ||
	    config->vm_start > GPU_M4_DRM_UAT_CONTEXT_START ||
	    config->vm_end < GPU_M4_DRM_UAT_PRIVATE_END || config->vm_end > (1ULL << 42) ||
	    !gpu_m4_drm_aligned(config->vm_start) || !gpu_m4_drm_aligned(config->vm_end) ||
	    config->memory_limit < GPU_M4_DRM_UAT_CONTEXT_SIZE + 3 * GPU_M4_DRM_PAGE_SIZE ||
	    config->removal_reserve > config->memory_limit ||
	    !config->max_leaves || config->max_leaves > GPU_M4_DRM_UAT_MAX_LEAVES - 4 ||
	    !config->max_tables || config->max_tables > GPU_M4_DRM_UAT_MAX_TABLES)
		return ERR_PTR(-EINVAL);
	p = kzalloc_obj(*p);
	if (!p)
		return ERR_PTR(-ENOMEM);
	ret = config->get(config->session);
	if (ret) {
		kfree(p);
		return ERR_PTR(ret);
	}
	if (config->lock) {
		ret = config->validate_lock ?
			config->validate_lock(config->session, config->lock) : -EINVAL;
		if (ret) {
			config->put(config->session);
			kfree(p);
			return ERR_PTR(ret);
		}
	}
	p->config = *config;
	mutex_init(&p->internal_lock);
	p->lock = config->lock ? config->lock : &p->internal_lock;
	refcount_set(&p->refs, 1);
	INIT_LIST_HEAD(&p->vms);
	INIT_LIST_HEAD(&p->garbage_versions);
	INIT_LIST_HEAD(&p->garbage_objects);
	INIT_LIST_HEAD(&p->garbage_vms);
	INIT_LIST_HEAD(&p->garbage_private);
	INIT_LIST_HEAD(&p->garbage_graphs);
	INIT_LIST_HEAD(&p->retained_graphs);
	INIT_LIST_HEAD(&p->retained_vms);
	spin_lock_init(&p->dead_lock);
	INIT_LIST_HEAD(&p->dead_objects);
	INIT_WORK(&p->tree_work, gpu_m4_drm_provider_tree_work);
	p->fault_revision = 1;
	return p;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_create);

int gpu_m4_drm_provider_get(struct gpu_m4_drm_provider *p)
{
	if (!p || !refcount_inc_not_zero(&p->refs))
		return -ENOENT;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_get);

/* Any ASID still names a resident or pending graph. */
static bool gpu_m4_drm_provider_bound_locked(struct gpu_m4_drm_provider *p)
{
	for (u32 asid = 1; asid < GPU_M4_DRM_PROVIDER_ASIDS; asid++)
		if (p->binding[asid].resident || p->binding[asid].pending)
			return true;
	return false;
}

int gpu_m4_drm_provider_put(struct gpu_m4_drm_provider *p)
{
	bool last;

	if (!p)
		return -EINVAL;
	mutex_lock(p->lock);
	if (refcount_read(&p->refs) == 1 &&
	    (!list_empty(&p->vms) || gpu_m4_drm_provider_bound_locked(p) || p->bytes ||
	     !list_empty(&p->garbage_versions) || !list_empty(&p->garbage_objects) ||
	     !list_empty(&p->garbage_vms) || !list_empty(&p->garbage_private) ||
	     !list_empty(&p->garbage_graphs) || !list_empty(&p->retained_graphs) ||
	     !list_empty(&p->retained_vms))) {
		mutex_unlock(p->lock);
		return -EBUSY;
	}
	last = refcount_dec_and_test(&p->refs);
	mutex_unlock(p->lock);
	if (last) {
		p->config.put(p->config.session);
		kfree(p);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_put);

struct mutex *gpu_m4_drm_provider_lock(struct gpu_m4_drm_provider *p)
{
	return p ? p->lock : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lock);

u64 gpu_m4_drm_provider_bytes_locked(struct gpu_m4_drm_provider *p)
{
	lockdep_assert_held(p->lock);
	return p->bytes;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_bytes_locked);

static bool gpu_m4_drm_provider_asid_valid(u32 asid)
{
	return asid && asid < GPU_M4_DRM_PROVIDER_ASIDS;
}

struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_asid_resident_locked(
	struct gpu_m4_drm_provider *p, u32 asid)
{
	lockdep_assert_held(p->lock);
	return gpu_m4_drm_provider_asid_valid(asid) ? p->binding[asid].resident : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_asid_resident_locked);

u64 gpu_m4_drm_provider_asid_generation_locked(struct gpu_m4_drm_provider *p, u32 asid)
{
	lockdep_assert_held(p->lock);
	return gpu_m4_drm_provider_asid_valid(asid) ? p->binding[asid].generation : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_asid_generation_locked);

struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_resident_locked(
	struct gpu_m4_drm_provider *p)
{
	return gpu_m4_drm_provider_asid_resident_locked(p, 1);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_resident_locked);

u64 gpu_m4_drm_provider_binding_generation_locked(struct gpu_m4_drm_provider *p)
{
	return gpu_m4_drm_provider_asid_generation_locked(p, 1);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_binding_generation_locked);

u8 gpu_m4_drm_provider_binding_wire_generation(u64 generation)
{
	return generation ? 1 + (generation - 1) % 255 : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_binding_wire_generation);

int gpu_m4_drm_provider_asid_quiescent_locked(struct gpu_m4_drm_provider *p, u32 asid)
{
	struct gpu_m4_drm_provider_binding *b;

	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_asid_valid(asid))
		return -EINVAL;
	if (p->error)
		return p->error;
	b = &p->binding[asid];
	return b->pending || (b->resident && b->resident->working) ? -EBUSY : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_asid_quiescent_locked);

static bool gpu_m4_drm_provider_vm_member(struct gpu_m4_drm_provider *p,
	const struct gpu_m4_drm_provider_vm *vm);
static struct gpu_m4_drm_provider_vm *gpu_m4_drm_provider_version_member(
	struct gpu_m4_drm_provider *p, const struct gpu_m4_drm_provider_version *v);

int gpu_m4_drm_provider_asid_binding_locked(struct gpu_m4_drm_provider *p, u32 asid,
	struct gpu_m4_drm_provider_version *candidate, u64 *generation, u8 *wire_generation)
{
	struct gpu_m4_drm_provider_binding *b;
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_provider_version *v;

	if (IS_ERR_OR_NULL(p) || IS_ERR_OR_NULL(candidate) || !generation || !wire_generation ||
	    !gpu_m4_drm_provider_asid_valid(asid))
		return -EINVAL;
	lockdep_assert_held(p->lock);
	if (p->error)
		return p->error;
	b = &p->binding[asid];
	if (b->pending)
		return -EBUSY;
	/* Do not dereference a caller-supplied identity before proving membership.
	 * A closed VM may still own accepted jobs and the resident baseline.
	 */
	if (b->resident != candidate || !b->generation)
		return -ESTALE;
	/* Hashed membership of the version, then of its VM: O(1) for any number
	 * of retained VMs, same proof as walking every VM's versions.
	 */
	vm = gpu_m4_drm_provider_version_member(p, candidate);
	if (!vm)
		return -ENOENT;
	/* A member version whose VM is not a listed VM is inconsistent. */
	if (!gpu_m4_drm_provider_vm_member(p, vm))
		return -ESTALE;
	v = candidate;
	if (!refcount_read(&vm->refs) || !refcount_read(&v->refs) ||
	    vm->provider != p || v->vm != vm || !v->protected ||
	    (v->state != GPU_M4_PROVIDER_VERSION_GRAPH &&
	     !gpu_m4_drm_provider_version_tree(v)) ||
	    IS_ERR_OR_NULL(vm->frontend) || IS_ERR_OR_NULL(v->snapshot) ||
	    (IS_ERR_OR_NULL(v->uat) && !gpu_m4_drm_provider_version_tree(v)) ||
	    !v->generation ||
	    gpu_m4_drm_vm_owner_vm(v->snapshot) != vm->frontend ||
	    gpu_m4_drm_vm_owner_generation(v->snapshot) != v->generation ||
	    v->binding_asid != asid)
		return -ESTALE;
	*generation = b->generation;
	*wire_generation = gpu_m4_drm_provider_binding_wire_generation(*generation);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_asid_binding_locked);

int gpu_m4_drm_provider_binding_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_version *candidate, u64 *generation, u8 *wire_generation)
{
	return gpu_m4_drm_provider_asid_binding_locked(p, 1, candidate, generation,
		wire_generation);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_binding_locked);

int gpu_m4_drm_provider_error_locked(struct gpu_m4_drm_provider *p)
{
	lockdep_assert_held(p->lock);
	return p->error;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_error_locked);

/* Provider mutex held: no lockless capture can take the withdrawn ticket. */
static void gpu_m4_drm_provider_vm_withdraw_locked(struct gpu_m4_drm_provider_vm *vm)
{
	lockdep_assert_held(vm->provider->lock);
	spin_lock(&vm->capture_lock);
	vm->capture = NULL;
	vm->capture_uat = NULL;
	spin_unlock(&vm->capture_lock);
}

/* Lockless sticky snapshot for status reports only, never an admission or
 * publication gate (those keep using the locked read).
 */
int gpu_m4_drm_provider_error(struct gpu_m4_drm_provider *p)
{
	return p ? READ_ONCE(p->error) : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_error);

/*
 * Shared-domain fail-stop: no further capture, publication or bind for ANY
 * VM. It does not itself latch every frontend VM status: each admitted job
 * this failure ends (in flight, lane-queued, scheduler-pending or a batch's
 * remaining windows) quarantines its own VM as it completes with the error,
 * and an idle VM learns of the failure from its VM_STATUS query (-EIO).
 */
void gpu_m4_drm_provider_fault_locked(struct gpu_m4_drm_provider *p, int error)
{
	struct gpu_m4_drm_provider_vm *vm;

	lockdep_assert_held(p->lock);
	if (!p->error)
		WRITE_ONCE(p->error, error < 0 ? error : -EIO);
	p->fault_revision++; /* fault boundary: next refresh is a full scan */
	list_for_each_entry(vm, &p->vms, link)
		gpu_m4_drm_provider_vm_withdraw_locked(vm);
	if (p->config.error_wake_locked)
		p->config.error_wake_locked(p, NULL);
	/* Root/job/cache owners remain reachable. No error fence is a halt proof. */
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fault_locked);

void gpu_m4_drm_provider_vm_quarantine_locked(struct gpu_m4_drm_provider_vm *vm, int error)
{
	if (!vm)
		return;
	lockdep_assert_held(vm->provider->lock);
	gpu_m4_drm_vm_error(vm->frontend, error < 0 ? error : -EIO);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_quarantine_locked);

int gpu_m4_drm_provider_vm_status_locked(struct gpu_m4_drm_provider_vm *vm)
{
	lockdep_assert_held(vm->provider->lock);
	return vm->error ?: gpu_m4_drm_vm_status(vm->frontend);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_status_locked);

int gpu_m4_drm_provider_vm_get_locked(struct gpu_m4_drm_provider_vm *vm)
{
	if (!vm)
		return -EINVAL;
	lockdep_assert_held(vm->provider->lock);
	return refcount_inc_not_zero(&vm->refs) ? 0 : -ENOENT;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_get_locked);

void gpu_m4_drm_provider_vm_put_locked(struct gpu_m4_drm_provider_vm *vm)
{
	lockdep_assert_held(vm->provider->lock);
	if (refcount_dec_and_test(&vm->refs))
		list_add_tail(&vm->garbage, &vm->provider->garbage_vms);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_put_locked);

struct gpu_m4_drm_provider *gpu_m4_drm_provider_vm_provider(
	struct gpu_m4_drm_provider_vm *vm)
{
	return vm ? vm->provider : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_provider);

bool gpu_m4_drm_provider_vm_open_locked(struct gpu_m4_drm_provider_vm *vm)
{
	lockdep_assert_held(vm->provider->lock);
	return !vm->closed && !vm->error && !vm->provider->error;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_open_locked);

/* This VM's sticky local failure (a removal its BO close could not commit for
 * lack of host memory), never the provider's. New work of the VM must stop
 * before it is protected; its retained and published owners are unaffected.
 */
int gpu_m4_drm_provider_vm_error_locked(struct gpu_m4_drm_provider_vm *vm)
{
	lockdep_assert_held(vm->provider->lock);
	return vm->error;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_error_locked);

struct gpu_m4_drm_provider_private_lease *gpu_m4_drm_provider_private_reserve_locked(
	struct gpu_m4_drm_provider_vm *vm, u64 va, u64 size,
	u64 guard_before, u64 guard_after)
{
	struct gpu_m4_drm_provider_private_lease *lease, *other;
	struct gpu_m4_drm_provider *p;
	u64 start, end;
	int ret;

	if (!vm || !gpu_m4_drm_aligned(va) || !gpu_m4_drm_aligned(size) ||
	    !gpu_m4_drm_aligned(guard_before) || !gpu_m4_drm_aligned(guard_after) ||
	    !gpu_m4_drm_range(va, size, GPU_M4_DRM_UAT_PRIVATE_START,
			      GPU_M4_DRM_UAT_PRIVATE_END) ||
	    guard_before > va - GPU_M4_DRM_UAT_PRIVATE_START ||
	    guard_after > GPU_M4_DRM_UAT_PRIVATE_END - va - size)
		return ERR_PTR(-EINVAL);
	p = vm->provider;
	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm))
		return ERR_PTR(p->error ? p->error : -ENOENT);
	if (vm->private_lease_count >= min(p->config.max_leaves, GPU_M4_DRM_MAX_BINDS))
		return ERR_PTR(-ENOSPC);
	start = va - guard_before;
	end = va + size + guard_after;
	list_for_each_entry(other, &vm->private_leases, link)
		if (start < other->end && other->start < end)
			return ERR_PTR(-EADDRINUSE);
	ret = gpu_m4_drm_provider_charge(p, sizeof(*lease));
	if (ret)
		return ERR_PTR(ret);
	lease = kzalloc_obj(*lease);
	if (!lease) {
		p->bytes -= sizeof(*lease);
		return ERR_PTR(-ENOMEM);
	}
	lease->vm = vm;
	lease->va = va;
	lease->size = size;
	lease->start = start;
	lease->end = end;
	refcount_set(&lease->refs, 1);
	INIT_LIST_HEAD(&lease->garbage);
	gpu_m4_drm_provider_vm_get_locked(vm);
	list_add_tail(&lease->link, &vm->private_leases);
	vm->private_lease_count++;
	return lease;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_private_reserve_locked);

struct gpu_m4_drm_provider_private_lease *gpu_m4_drm_provider_private_adopt_locked(
	struct gpu_m4_drm_provider_vm *vm, u64 va, u64 size,
	u64 guard_before, u64 guard_after)
{
	struct gpu_m4_drm_provider_private_lease *other, *found = NULL;
	struct gpu_m4_drm_provider *p;
	u64 start, end;
	int ret;

	if (!vm || !gpu_m4_drm_aligned(va) || !gpu_m4_drm_aligned(size) ||
	    !gpu_m4_drm_aligned(guard_before) || !gpu_m4_drm_aligned(guard_after) ||
	    !gpu_m4_drm_range(va, size, GPU_M4_DRM_UAT_PRIVATE_START,
			      GPU_M4_DRM_UAT_PRIVATE_END) ||
	    guard_before > va - GPU_M4_DRM_UAT_PRIVATE_START ||
	    guard_after > GPU_M4_DRM_UAT_PRIVATE_END - va - size)
		return ERR_PTR(-EINVAL);
	p = vm->provider;
	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm))
		return ERR_PTR(p->error ? p->error : -ENOENT);
	start = va - guard_before;
	end = va + size + guard_after;
	list_for_each_entry(other, &vm->private_leases, link) {
		if (!(start < other->end && other->start < end))
			continue;
		if (found || other->va != va || other->size != size || other->start != start ||
		    other->end != end || !refcount_read(&other->refs))
			return ERR_PTR(-EADDRINUSE);
		found = other;
	}
	if (!found)
		return ERR_PTR(-ENOENT);
	ret = gpu_m4_drm_provider_private_get_locked(found);
	return ret ? ERR_PTR(ret) : found;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_private_adopt_locked);

int gpu_m4_drm_provider_private_find_locked(struct gpu_m4_drm_provider_vm *vm,
	u64 minimum_va, u64 size, u64 alignment, u64 guard_before, u64 guard_after,
	u64 *va)
{
	const u64 low = GPU_M4_DRM_UAT_PRIVATE_START, high = GPU_M4_DRM_UAT_PRIVATE_END;
	struct gpu_m4_drm_provider_private_lease *other;
	u64 candidate, limit;

	if (!vm || !va || !gpu_m4_drm_aligned(minimum_va) ||
	    minimum_va < low || minimum_va > high || !size || !gpu_m4_drm_aligned(size) ||
	    !gpu_m4_drm_aligned(guard_before) || !gpu_m4_drm_aligned(guard_after) ||
	    alignment < GPU_M4_DRM_PAGE_SIZE || alignment > high - low ||
	    (alignment & (alignment - 1)))
		return -EINVAL;
	lockdep_assert_held(vm->provider->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm))
		return vm->provider->error ?: -ENOENT;
	if (guard_before > high - low || size > high - low - guard_before ||
	    guard_after > high - low - guard_before - size)
		return -ENOSPC;
	limit = high - size - guard_after;
	candidate = ALIGN(max(minimum_va, low + guard_before), alignment);
	/* Each advance passes the end of at least one distinct held interval.
	 * The list is deliberately not assumed sorted; guards are claim bounds.
	 */
	for (u32 skipped = 0; skipped <= vm->private_lease_count; skipped++) {
		u64 next = candidate;

		if (candidate > limit)
			return -ENOSPC;
		list_for_each_entry(other, &vm->private_leases, link)
			if (candidate - guard_before < other->end &&
			    other->start < candidate + size + guard_after)
				next = max(next, ALIGN(other->end + guard_before, alignment));
		if (next == candidate) {
			*va = candidate;
			return 0;
		}
		candidate = next;
	}
	return -ENOSPC;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_private_find_locked);

int gpu_m4_drm_provider_private_check_locked(struct gpu_m4_drm_provider_private_lease *lease,
	struct gpu_m4_drm_provider_vm *vm, u64 va, u64 size)
{
	if (!vm)
		return -EINVAL;
	lockdep_assert_held(vm->provider->lock);
	if (!lease || lease->vm != vm || !refcount_read(&lease->refs))
		return -EPERM;
	if (!gpu_m4_drm_aligned(va) || !gpu_m4_drm_aligned(size) ||
	    !gpu_m4_drm_range(va, size, lease->va, lease->va + lease->size))
		return -EINVAL;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_private_check_locked);

int gpu_m4_drm_provider_private_get_locked(struct gpu_m4_drm_provider_private_lease *lease)
{
	if (!lease)
		return -EINVAL;
	lockdep_assert_held(lease->vm->provider->lock);
	return refcount_inc_not_zero(&lease->refs) ? 0 : -ENOENT;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_private_get_locked);

void gpu_m4_drm_provider_private_put_locked(struct gpu_m4_drm_provider_private_lease *lease)
{
	struct gpu_m4_drm_provider_vm *vm = lease->vm;

	lockdep_assert_held(vm->provider->lock);
	if (refcount_dec_and_test(&lease->refs)) {
		list_del_init(&lease->link);
		vm->private_lease_count--;
		list_add_tail(&lease->garbage, &vm->provider->garbage_private);
	}
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_private_put_locked);

/* Read-only: is this EXACT registered address claim held only by its caller's
 * one reference? The caller holds a reference to the lease (its BO's claim),
 * so the lease itself is live memory. Membership of the VM in the provider
 * precedes every VM dereference. A lease with a nonzero count is linked on
 * its own VM's live claim list from reserve until its final put (the only
 * unlink), so a live count, a linked node and the VM back-pointer are its
 * registration: no O(claims) walk of that list (this runs for every pending
 * cold cell on every reap pass, and a closed client's VM can hold thousands
 * of claims). Then VA, size and both guards must match. Captured versions,
 * open or closed VM alike, are other holders (-EBUSY). Never a reservation,
 * transfer or refcount change.
 */
int gpu_m4_drm_provider_private_sole_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_vm *vm, struct gpu_m4_drm_provider_private_lease *lease,
	u64 va, u64 size, u64 guard_before, u64 guard_after)
{
	struct gpu_m4_drm_provider_vm *found = NULL;

	if (IS_ERR_OR_NULL(p) || IS_ERR_OR_NULL(vm) || IS_ERR_OR_NULL(lease))
		return -EINVAL;
	lockdep_assert_held(p->lock);
	if (p->error)
		return p->error;
	if (gpu_m4_drm_provider_vm_member(p, vm))
		found = vm;
	if (!found || !refcount_read(&found->refs) || found->provider != p)
		return -ENOENT;
	if (!found->private_lease_count)
		return -ESTALE;
	if (!refcount_read(&lease->refs) || lease->vm != found || list_empty(&lease->link))
		return -ENOENT;
	if (lease->vm != found || lease->va != va || lease->size != size ||
	    va < guard_before || lease->start != va - guard_before ||
	    size > U64_MAX - va || guard_after > U64_MAX - va - size ||
	    lease->end != va + size + guard_after)
		return -EPERM;
	return refcount_read(&lease->refs) == 1 ? 0 : -EBUSY;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_private_sole_locked);

static int gpu_m4_drm_provider_context_get(void *cookie, struct gpu_m4_drm_vm *vm,
					  const void *identity)
{
	struct gpu_m4_drm_provider_context *c = cookie;

	if (identity != c || vm != c->vm->frontend || !c->nc)
		return -EPERM;
	refcount_inc(&c->refs);
	return 0;
}

static void gpu_m4_drm_provider_context_put(void *cookie)
{
	struct gpu_m4_drm_provider_context *c = cookie;

	/* The VM's baseline outlives every version; this cannot be the final put. */
	refcount_dec(&c->refs);
}

static int gpu_m4_drm_provider_context_page(void *cookie, u64 offset, u64 *pa)
{
	struct gpu_m4_drm_provider_context *c = cookie;
	struct gpu_m4_drm_provider_vm *vm = c->vm;

	if (!pa || !gpu_m4_drm_aligned(offset) || offset >= GPU_M4_DRM_UAT_CONTEXT_SIZE ||
	    !c->page[offset / GPU_M4_DRM_PAGE_SIZE])
		return -EINVAL;
	*pa = page_to_phys(c->page[offset / GPU_M4_DRM_PAGE_SIZE]);
	return vm->provider->config.validate_page(vm->provider->config.session,
		vm->frontend, c, offset, *pa);
}

static void gpu_m4_drm_provider_context_free(struct gpu_m4_drm_provider_context *c)
{
	if (c->nc)
		vunmap(c->nc);
	for (u32 i = 0; i < ARRAY_SIZE(c->page); i++)
		if (c->page[i])
			__free_pages(c->page[i], 0);
}

static int gpu_m4_drm_provider_context_alloc(struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider_context *c = &vm->context;

	if (PAGE_SIZE != GPU_M4_DRM_PAGE_SIZE)
		return -EOPNOTSUPP;
	c->vm = vm;
	refcount_set(&c->refs, 1);
	for (u32 i = 0; i < ARRAY_SIZE(c->page); i++) {
		unsigned long cpu;

		c->page[i] = alloc_pages(GFP_KERNEL | __GFP_ZERO, 0);
		if (!c->page[i])
			return -ENOMEM;
		cpu = (unsigned long)page_address(c->page[i]);
		if (!cpu)
			return -EPERM;
		/* One-time WB zero initialization is completed before the NC view.
		 * No subsequent CPU access to the WB direct-map alias is allowed.
		 */
		dcache_clean_poc(cpu, cpu + PAGE_SIZE);
		dcache_inval_poc(cpu, cpu + PAGE_SIZE);
	}
	c->nc = vmap(c->page, ARRAY_SIZE(c->page), VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	return c->nc ? 0 : -ENOMEM;
}

static struct gpu_m4_drm_provider_object **gpu_m4_drm_provider_object_bucket(
	struct gpu_m4_drm_provider_vm *vm, const void *identity)
{
	u64 key = (unsigned long)identity;

	return &vm->object_hash[(key * 0x9e3779b97f4a7c15ULL) >>
				(64 - GPU_M4_DRM_PROVIDER_OBJECT_HASH_BITS)];
}

/* Identity index of the VM's live registered objects (exactly vm->objects):
 * inserted with the list link at admission commit, removed with it at the
 * zero-ref transition. Callers hold the provider lock.
 */
static struct gpu_m4_drm_provider_object *gpu_m4_drm_provider_object_find(
	struct gpu_m4_drm_provider_vm *vm, const void *identity)
{
	struct gpu_m4_drm_provider_object *o;

	lockdep_assert_held(vm->provider->lock);
	for (o = *gpu_m4_drm_provider_object_bucket(vm, identity); o; o = o->hash_next)
		if (o->backing.identity == identity)
			return o;
	return NULL;
}

static u32 gpu_m4_drm_provider_member_slot(const void *key)
{
	return ((u64)(unsigned long)key * 0x9e3779b97f4a7c15ULL) >> 56;
}

/* p->vms membership by address only (never dereferences vm). */
static bool gpu_m4_drm_provider_vm_member(struct gpu_m4_drm_provider *p,
	const struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider_vm *each;

	for (each = p->member_vm[gpu_m4_drm_provider_member_slot(vm)]; each;
	     each = each->member_next)
		if (each == vm)
			return true;
	return false;
}

static void gpu_m4_drm_provider_vm_list_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider_vm **bucket =
		&p->member_vm[gpu_m4_drm_provider_member_slot(vm)];

	list_add_tail(&vm->link, &p->vms);
	vm->member_next = *bucket;
	*bucket = vm;
}

static void gpu_m4_drm_provider_vm_unlist_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider_vm **link;

	list_del(&vm->link);
	for (link = &p->member_vm[gpu_m4_drm_provider_member_slot(vm)]; *link;
	     link = &(*link)->member_next) {
		if (*link == vm) {
			*link = vm->member_next;
			break;
		}
	}
	vm->member_next = NULL;
}

/* Membership precedes dereference: only proven bucket members are read. */
static struct gpu_m4_drm_provider_vm *gpu_m4_drm_provider_object_member(
	struct gpu_m4_drm_provider *p, const struct gpu_m4_drm_provider_object *o)
{
	struct gpu_m4_drm_provider_object *each;

	for (each = p->member_object[gpu_m4_drm_provider_member_slot(o)]; each;
	     each = each->member_next)
		if (each == o)
			return each->vm;
	return NULL;
}

static struct gpu_m4_drm_provider_vm *gpu_m4_drm_provider_version_member(
	struct gpu_m4_drm_provider *p, const struct gpu_m4_drm_provider_version *v)
{
	struct gpu_m4_drm_provider_version *each;

	for (each = p->member_version[gpu_m4_drm_provider_member_slot(v)]; each;
	     each = each->member_next)
		if (each == v)
			return each->vm;
	return NULL;
}

static void gpu_m4_drm_provider_version_link(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_version *v)
{
	struct gpu_m4_drm_provider_version **bucket =
		&vm->provider->member_version[gpu_m4_drm_provider_member_slot(v)];

	list_add_tail(&v->link, &vm->versions);
	v->member_next = *bucket;
	*bucket = v;
	v->membered = true;
}

static void gpu_m4_drm_provider_version_unlink(struct gpu_m4_drm_provider_version *v)
{
	struct gpu_m4_drm_provider_version **link;

	list_del(&v->link);
	if (!v->membered)
		return;
	for (link = &v->vm->provider->member_version[gpu_m4_drm_provider_member_slot(v)];
	     *link; link = &(*link)->member_next)
		if (*link == v) {
			*link = v->member_next;
			break;
		}
	v->member_next = NULL;
	v->membered = false;
}

static void gpu_m4_drm_provider_object_register(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_object *o)
{
	struct gpu_m4_drm_provider_object **bucket =
		gpu_m4_drm_provider_object_bucket(vm, o->backing.identity);

	lockdep_assert_held(vm->provider->lock);
	list_add_tail(&o->link, &vm->objects);
	spin_lock(&vm->object_lock);
	o->hash_next = *bucket;
	*bucket = o;
	spin_unlock(&vm->object_lock);
	o->member_next = vm->provider->member_object[gpu_m4_drm_provider_member_slot(o)];
	vm->provider->member_object[gpu_m4_drm_provider_member_slot(o)] = o;
	o->hashed = true;
}

static void gpu_m4_drm_provider_object_unregister(struct gpu_m4_drm_provider_object *o)
{
	struct gpu_m4_drm_provider_object **link;

	list_del_init(&o->link);
	if (!o->hashed)
		return; /* A rejected/duplicate stage was never registered. */
	spin_lock(&o->vm->object_lock);
	for (link = gpu_m4_drm_provider_object_bucket(o->vm, o->backing.identity);
	     *link; link = &(*link)->hash_next) {
		if (*link == o) {
			*link = o->hash_next;
			break;
		}
	}
	spin_unlock(&o->vm->object_lock);
	for (link = &o->vm->provider->member_object[gpu_m4_drm_provider_member_slot(o)];
	     *link; link = &(*link)->member_next) {
		if (*link == o) {
			*link = o->member_next;
			break;
		}
	}
	o->hash_next = o->member_next = NULL;
	o->hashed = false;
}

/* Exact cached admission for this retained pin and offset. Returns 0 when the
 * admission applies AND the session's live gate accepts, a positive value when
 * the full oracle is required (no gate, provider fault latched, or any key
 * drift), or the gate's negative error.
 */
static int gpu_m4_drm_provider_object_admitted(struct gpu_m4_drm_provider_object *o,
	u64 offset, u64 pa)
{
	struct gpu_m4_drm_provider *p = o->vm->provider;
	const struct gpu_m4_drm_provider_admission *a = &o->admission;

	lockdep_assert_held(p->lock);
	if (!a->admitted || !p->config.admitted_page || p->error ||
	    a->session != p->config.session || a->identity != o->backing.identity ||
	    a->pin != o->pin || a->pa != o->backing.pa || a->size != o->backing.size ||
	    a->cpu_policy != o->backing.cpu_policy ||
	    gpu_m4_drm_bo_policy((struct drm_gem_object *)o->backing.identity) != a->cpu_policy ||
	    !gpu_m4_drm_aligned(offset) || offset >= a->size ||
	    a->pa[offset / GPU_M4_DRM_PAGE_SIZE] != pa)
		return 1;
	return p->config.admitted_page(p->config.session, o->vm->frontend,
				       o->backing.identity, offset);
}

/* Page oracle for a registered object's held pin: cached admission plus the
 * live gate, else the session's complete validate_page.
 */
static int gpu_m4_drm_provider_object_oracle(struct gpu_m4_drm_provider_object *o,
	u64 offset, u64 pa)
{
	struct gpu_m4_drm_provider *p = o->vm->provider;
	int ret = gpu_m4_drm_provider_object_admitted(o, offset, pa);

	if (ret <= 0)
		return ret;
	return p->config.validate_page(p->config.session, o->vm->frontend,
				       o->backing.identity, offset, pa);
}

int gpu_m4_drm_provider_object_get_locked(struct gpu_m4_drm_provider_object *o)
{
	if (!o)
		return -EINVAL;
	lockdep_assert_held(o->vm->provider->lock);
	return refcount_inc_not_zero(&o->refs) ? 0 : -ENOENT;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_object_get_locked);

void gpu_m4_drm_provider_object_put_locked(struct gpu_m4_drm_provider_object *o)
{
	lockdep_assert_held(o->vm->provider->lock);
	if (refcount_dec_and_test(&o->refs)) {
		gpu_m4_drm_provider_object_unregister(o);
		list_add_tail(&o->garbage, &o->vm->provider->garbage_objects);
	}
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_object_put_locked);

/* ---- S3c: references dropped without the provider (GPU) mutex ---- */

/* REMOVE-DEBUG: authority VMs, binds, ops, first admissions (the only bind
 * step that takes the provider mutex), unlocked GEM closes, deferred final
 * puts and worker passes.
 */
enum {
	GPU_M4_DRM_PROVIDER_TREE_VMS,
	GPU_M4_DRM_PROVIDER_TREE_BINDS,
	GPU_M4_DRM_PROVIDER_TREE_BIND_OPS,
	GPU_M4_DRM_PROVIDER_TREE_BIND_REFUSED,
	GPU_M4_DRM_PROVIDER_TREE_ADMISSIONS,
	GPU_M4_DRM_PROVIDER_TREE_BO_CLOSES,
	GPU_M4_DRM_PROVIDER_TREE_DEAD_PUTS,
	GPU_M4_DRM_PROVIDER_TREE_WORK,
	GPU_M4_DRM_PROVIDER_TREE_MEMO_HITS,
	GPU_M4_DRM_PROVIDER_TREE_MEMO_LOG_HITS,
	GPU_M4_DRM_PROVIDER_TREE_MEMO_MISSES,
	GPU_M4_DRM_PROVIDER_TREE_MEMO_STALE,
	GPU_M4_DRM_PROVIDER_TREE_STATS,
};

static const char *const gpu_m4_drm_provider_tree_stat_name[GPU_M4_DRM_PROVIDER_TREE_STATS] = {
	"vms", "binds", "bind_ops", "bind_refused", "admissions", "bo_closes", "dead_puts", "work",
	"memo_hits", "memo_log_hits", "memo_misses", "memo_stale",
};

static atomic64_t gpu_m4_drm_provider_tree_stat[GPU_M4_DRM_PROVIDER_TREE_STATS];

static void gpu_m4_drm_provider_tree_count(u32 stat, u64 value)
{
	atomic64_add(value, &gpu_m4_drm_provider_tree_stat[stat]);
}

static int __maybe_unused gpu_m4_drm_provider_tree_stats_get(char *buffer,
	const struct kernel_param *kp __always_unused)
{
	int at = 0;

	for (u32 i = 0; i < GPU_M4_DRM_PROVIDER_TREE_STATS; i++)
		at += scnprintf(buffer + at, PAGE_SIZE - at, "%s%s=%lld", i ? " " : "",
				gpu_m4_drm_provider_tree_stat_name[i],
				(long long)atomic64_read(&gpu_m4_drm_provider_tree_stat[i]));
	at += scnprintf(buffer + at, PAGE_SIZE - at, "\n");
	return at;
}

/* REMOVE-DEBUG: debugfs gpu_m4_drm_provider_vm/authority_stats (live counters are not module parameters). */

/*
 * S3c gate, hardware-qualified (J713: desktop, Firefox WebGL, SuperTuxKart,
 * with user_asids and two packets in flight). 1 (default): each new
 * ordinary user VM's persistent tree is its hardware authority (TTBAT names
 * its root; VM_BIND and GEM close edit it with real TLBI; jobs hold a guard
 * and the VM's TREE version instead of a captured snapshot/graph). 0: the
 * old version path (the shadow tree per gpu_m4_drm_vm_tree.mode), kept as
 * a rollback until S3d deletes it. Sampled once per VM at creation:
 * existing VMs keep the path they were created with.
 */
static bool vm_tree_authority = true;
module_param(vm_tree_authority, bool, 0644);
MODULE_PARM_DESC(vm_tree_authority, "S3c: new ordinary VMs use their persistent page-table tree as the hardware authority (default 1; 0 = version path rollback)");

/* Queue tree_work holding a provider reference (any context). */
static void gpu_m4_drm_provider_tree_kick(struct gpu_m4_drm_provider *p)
{
	if (gpu_m4_drm_provider_get(p))
		return;
	/* Already pending: that pass sees this request; drop the extra
	 * reference without the mutex (never the last: the requester's owner
	 * still holds one).
	 */
	if (!queue_work(system_unbound_wq, &p->tree_work))
		WARN_ON(!refcount_dec_not_one(&p->refs));
}

/*
 * Drop a reference without the provider mutex. The final transition still
 * happens under it: the last reference is handed to tree_work, which puts
 * it (unregister, garbage) and reaps. Pins therefore always outlive the
 * tree's removal sequence that dropped them (I2).
 */
static void gpu_m4_drm_provider_object_put_deferred(struct gpu_m4_drm_provider_object *o)
{
	struct gpu_m4_drm_provider *p;
	unsigned long flags;

	if (IS_ERR_OR_NULL(o) || refcount_dec_not_one(&o->refs))
		return;
	p = o->vm->provider;
	spin_lock_irqsave(&p->dead_lock, flags);
	list_add_tail(&o->dead, &p->dead_objects);
	spin_unlock_irqrestore(&p->dead_lock, flags);
	gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_DEAD_PUTS, 1);
	gpu_m4_drm_provider_tree_kick(p);
}

/* A registered live object of this VM, with a reference, without the mutex. */
static struct gpu_m4_drm_provider_object *gpu_m4_drm_provider_object_lookup(
	struct gpu_m4_drm_provider_vm *vm, const void *identity)
{
	struct gpu_m4_drm_provider_object *o;

	spin_lock(&vm->object_lock);
	for (o = *gpu_m4_drm_provider_object_bucket(vm, identity); o; o = o->hash_next)
		if (o->backing.identity == identity)
			break;
	if (o && !refcount_inc_not_zero(&o->refs))
		o = NULL;
	spin_unlock(&vm->object_lock);
	return o;
}

struct gpu_m4_drm_bo_pin *gpu_m4_drm_provider_object_native(
	struct gpu_m4_drm_provider_object *o)
{
	return o ? o->pin : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_object_native);

struct drm_gem_object *gpu_m4_drm_provider_object_identity(
	struct gpu_m4_drm_provider_object *o)
{
	return o ? (struct drm_gem_object *)o->backing.identity : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_object_identity);

int gpu_m4_drm_provider_object_page_locked(struct gpu_m4_drm_provider_object *o,
	u64 offset, u64 *pa)
{
	struct gpu_m4_drm_provider *p = o->vm->provider;
	u64 actual;
	int ret;

	lockdep_assert_held(p->lock);
	if (!pa || !gpu_m4_drm_aligned(offset) || offset >= o->backing.size ||
	    !refcount_read(&o->refs))
		return -EINVAL;
	ret = gpu_m4_drm_bo_pin_page(o->pin, offset, &actual);
	if (ret || actual != o->backing.pa[offset / GPU_M4_DRM_PAGE_SIZE])
		return ret ? ret : -EPROTO;
	/* Native backing consumers share the pin's admitted inventory. */
	ret = gpu_m4_drm_provider_object_oracle(o, offset, actual);
	if (!ret)
		*pa = actual;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_object_page_locked);

static bool gpu_m4_drm_provider_object_origin(struct gpu_m4_drm_provider_object *o,
	struct gpu_m4_drm_provider_vm *vm)
{
	return o && o->vm == vm && refcount_read(&o->refs) && o->pin &&
		gpu_m4_drm_bo_pin_object(o->pin) == o->backing.identity &&
		gpu_m4_drm_bo_pin_size(o->pin) == o->backing.size &&
		gpu_m4_drm_bo_belongs_to_vm((struct drm_gem_object *)o->backing.identity, vm->frontend) &&
		gpu_m4_drm_bo_policy((struct drm_gem_object *)o->backing.identity) == o->backing.cpu_policy;
}

int gpu_m4_drm_provider_object_origin_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_version *captured,
	struct gpu_m4_drm_provider_object *object)
{
	struct gpu_m4_drm_provider_vm *version_vm = NULL, *object_vm = NULL;

	if (IS_ERR_OR_NULL(p) || IS_ERR_OR_NULL(captured) || IS_ERR_OR_NULL(object))
		return -EINVAL;
	lockdep_assert_held(p->lock);
	if (!refcount_read(&p->refs))
		return -ENOENT;
	if (p->error)
		return p->error;
	/* Membership precedes every dereference of either supplied identity. An
	 * object need not occur in captured->objects or its lower-VA ledger.
	 */
	version_vm = gpu_m4_drm_provider_version_member(p, captured);
	object_vm = gpu_m4_drm_provider_object_member(p, object);
	if (version_vm && !gpu_m4_drm_provider_vm_member(p, version_vm))
		version_vm = NULL;
	if (object_vm && !gpu_m4_drm_provider_vm_member(p, object_vm))
		object_vm = NULL;
	if (!version_vm || !object_vm || !refcount_read(&captured->refs) ||
	    !refcount_read(&object->refs) || !refcount_read(&version_vm->refs) ||
	    !refcount_read(&object_vm->refs))
		return -ENOENT;
	if (object_vm != version_vm || version_vm->provider != p ||
	    captured->vm != version_vm || object->vm != version_vm)
		return -EPERM;
	if (!captured->generation || captured->state < GPU_M4_PROVIDER_VERSION_LOGICAL ||
	    (captured->state >= GPU_M4_PROVIDER_VERSION_SNAPSHOT &&
	     (IS_ERR_OR_NULL(captured->snapshot) ||
	      gpu_m4_drm_vm_owner_generation(captured->snapshot) != captured->generation ||
	      gpu_m4_drm_vm_owner_vm(captured->snapshot) != version_vm->frontend)))
		return -ESTALE;
	if (IS_ERR_OR_NULL(object->pin) || !object->backing.identity ||
	    !object->backing.pa || !object->backing.size ||
	    !gpu_m4_drm_aligned(object->backing.size) ||
	    (object->backing.cpu_policy != GPU_M4_DRM_CPU_NC &&
	     object->backing.cpu_policy != GPU_M4_DRM_CPU_WRITEBACK) ||
	    !gpu_m4_drm_provider_object_origin(object, version_vm))
		return -EPERM;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_object_origin_locked);

static int gpu_m4_drm_provider_native_get(void *cookie, struct gpu_m4_drm_vm *frontend,
	const void *identity)
{
	struct gpu_m4_drm_provider_object *o = cookie;

	lockdep_assert_held(o->vm->provider->lock);
	if (frontend != o->vm->frontend || identity != o->backing.identity ||
	    !gpu_m4_drm_provider_object_origin(o, o->vm))
		return -EPERM;
	return gpu_m4_drm_provider_object_get_locked(o);
}

static void gpu_m4_drm_provider_native_put(void *cookie)
{
	struct gpu_m4_drm_provider_object *o = cookie;
	struct gpu_m4_drm_provider *p = o->vm->provider;

	/* The version's inventory baseline outlives UAT callbacks and snapshots.
	 * Queue only the reference drop; native destruction happens in reap.
	 */
	lockdep_assert_not_held(p->lock);
	mutex_lock(p->lock);
	gpu_m4_drm_provider_object_put_locked(o);
	mutex_unlock(p->lock);
}

static int gpu_m4_drm_provider_native_page(void *cookie, u64 offset, u64 *pa)
{
	return gpu_m4_drm_provider_object_page_locked(cookie, offset, pa);
}

static struct gpu_m4_drm_provider_object *gpu_m4_drm_provider_object_pin_owned(
	struct gpu_m4_drm_provider_vm *vm, struct drm_gem_object *object, bool defer_reap)
{
	struct gpu_m4_drm_provider *p;
	struct gpu_m4_drm_provider_object *o, *existing;
	u64 pages, charge;
	u64 *pa;
	int ret;

	if (!vm || !object || !object->resv || !object->size ||
	    !gpu_m4_drm_aligned(object->size) || PAGE_SIZE != GPU_M4_DRM_PAGE_SIZE)
		return ERR_PTR(-EINVAL);
	if (!gpu_m4_drm_bo_belongs_to_vm(object, vm->frontend))
		return ERR_PTR(-EPERM);
	p = vm->provider;
	lockdep_assert_not_held(p->lock);
	pages = object->size / GPU_M4_DRM_PAGE_SIZE;
	if (pages > p->config.max_leaves || pages > (U64_MAX - object->size) / sizeof(u64))
		return ERR_PTR(-E2BIG);
	/* This VM's own bookkeeping only: the record, its PA inventory and the
	 * native-pin wrapper as a conservative bounded allowance. The backing
	 * pages are the GEM's, paid once by its factory and shared by every VM
	 * (and dma-buf importer) pinning it; charging them per VM would count
	 * one buffer once per client.
	 */
	charge = pages * sizeof(u64) + sizeof(*o) + 0x100;
	mutex_lock(p->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm)) {
		ret = p->error ? p->error : -ENOENT;
		goto unlock_error;
	}
	existing = gpu_m4_drm_provider_object_find(vm, object);
	if (existing) {
		gpu_m4_drm_provider_object_get_locked(existing);
		mutex_unlock(p->lock);
		return existing;
	}
	ret = gpu_m4_drm_provider_charge(p, charge);
	if (ret)
		goto unlock_error;
	gpu_m4_drm_provider_vm_get_locked(vm);
	mutex_unlock(p->lock);
	o = kzalloc_obj(*o);
	pa = kvcalloc(pages, sizeof(*pa), GFP_KERNEL);
	if (!o || !pa) {
		ret = -ENOMEM;
		goto free_stage;
	}
	o->vm = vm;
	o->charge = charge;
	INIT_LIST_HEAD(&o->link);
	INIT_LIST_HEAD(&o->garbage);
	refcount_set(&o->refs, 1);
	ret = dma_resv_lock(object->resv, NULL);
	if (ret)
		goto free_stage;
	o->pin = gpu_m4_drm_bo_pin_locked(object);
	dma_resv_unlock(object->resv);
	if (IS_ERR(o->pin)) {
		ret = PTR_ERR(o->pin);
		o->pin = NULL;
		goto free_stage;
	}
	if (!o->pin || gpu_m4_drm_bo_pin_object(o->pin) != object ||
	    gpu_m4_drm_bo_pin_size(o->pin) != object->size) {
		ret = -EPROTO;
		goto free_stage;
	}
	o->backing = (struct gpu_m4_drm_vm_backing) {
		.identity = object, .pa = pa, .size = object->size,
		.cpu_policy = gpu_m4_drm_bo_policy(object),
	};
	if (o->backing.cpu_policy != GPU_M4_DRM_CPU_NC &&
	    o->backing.cpu_policy != GPU_M4_DRM_CPU_WRITEBACK) {
		ret = -EOPNOTSUPP;
		goto free_stage;
	}
	for (u64 i = 0; i < pages; i++) {
		ret = gpu_m4_drm_bo_pin_page(o->pin, i * GPU_M4_DRM_PAGE_SIZE, &pa[i]);
		if (ret)
			goto free_stage;
		if (!pa[i] || !gpu_m4_drm_aligned(pa[i]) || (pa[i] & ~GPU_M4_PA_MASK)) {
			ret = -EPERM;
			goto free_stage;
		}
	}
	/* Pin, PA inventory and policy were constructed above outside every
	 * global lock. The commit below is the checked first admission.
	 */
	mutex_lock(p->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm)) {
		ret = p->error ? p->error : -ENOENT;
		goto unlock_stage;
	}
	existing = gpu_m4_drm_provider_object_find(vm, object);
	if (existing) {
		/* A racing first pin already admitted this identity; this stage
		 * is discarded and only its identical inventory is checked.
		 */
		if (existing->backing.size != o->backing.size ||
		    existing->backing.cpu_policy != o->backing.cpu_policy ||
		    memcmp(existing->backing.pa, pa, pages * sizeof(*pa))) {
			ret = -EPROTO;
			goto unlock_stage;
		}
		gpu_m4_drm_provider_object_get_locked(existing);
		mutex_unlock(p->lock);
		ret = 0;
		goto free_stage;
	}
	/* First admission validates every native PA of this exact pin once. */
	for (u64 i = 0; i < pages; i++) {
		ret = p->config.validate_page(p->config.session, vm->frontend,
					    object, i * GPU_M4_DRM_PAGE_SIZE, pa[i]);
		if (ret)
			goto unlock_stage;
	}
	o->admission = (struct gpu_m4_drm_provider_admission) {
		.session = p->config.session, .identity = object, .pin = o->pin,
		.pa = pa, .size = object->size, .cpu_policy = o->backing.cpu_policy,
		.admitted = true,
	};
	gpu_m4_drm_provider_object_register(vm, o);
	mutex_unlock(p->lock);
	return o;
unlock_stage:
	mutex_unlock(p->lock);
free_stage:
	if (defer_reap && o && o->pin) {
		/* An unpublished rejected/duplicate pin still owns its VM baseline,
		 * charge and PA allocation. Use the ordinary zero-ref garbage owner;
		 * never final-put it under the frontend's accepting serialization.
		 * Its self-linked inventory node was never admitted to vm->objects.
		 */
		o->backing.pa = pa;
		mutex_lock(p->lock);
		gpu_m4_drm_provider_object_put_locked(o);
		mutex_unlock(p->lock);
		return ret ? ERR_PTR(ret) : existing;
	}
	if (o && o->pin)
		gpu_m4_drm_bo_pin_put(o->pin);
	kvfree(pa);
	kfree(o);
	mutex_lock(p->lock);
	p->bytes -= charge;
	gpu_m4_drm_provider_vm_put_locked(vm);
	mutex_unlock(p->lock);
	if (!defer_reap)
		gpu_m4_drm_provider_reap(p);
	return ret ? ERR_PTR(ret) : existing;
unlock_error:
	mutex_unlock(p->lock);
	return ERR_PTR(ret);
}

struct gpu_m4_drm_provider_object *gpu_m4_drm_provider_object_pin(
	struct gpu_m4_drm_provider_vm *vm, struct drm_gem_object *object)
{
	return gpu_m4_drm_provider_object_pin_owned(vm, object, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_object_pin);

struct gpu_m4_drm_provider_object *gpu_m4_drm_provider_object_pin_deferred(
	struct gpu_m4_drm_provider_vm *vm, struct drm_gem_object *object)
{
	return gpu_m4_drm_provider_object_pin_owned(vm, object, true);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_object_pin_deferred);

static struct gpu_m4_drm_provider_object *gpu_m4_drm_provider_version_object(
	struct gpu_m4_drm_provider_version *v, const void *identity)
{
	u32 first = 0, end = v->object_count;

	/* Mutation is confined to a detached candidate. Its inventory is sorted
	 * once before any per-leaf oracle, then remains immutable while captured.
	 */
	if (v->objects_indexed) {
		while (first < end) {
			u32 middle = first + (end - first) / 2;
			struct gpu_m4_drm_provider_object *o = v->objects[middle];

			if (o->backing.identity == identity)
				return o;
			if ((unsigned long)o->backing.identity < (unsigned long)identity)
				first = middle + 1;
			else
				end = middle;
		}
		return NULL;
	}
	for (u32 i = 0; i < v->object_count; i++)
		if (v->objects[i]->backing.identity == identity)
			return v->objects[i];
	return NULL;
}

static int gpu_m4_drm_provider_object_compare(const void *first, const void *second)
{
	const struct gpu_m4_drm_provider_object *a = *(struct gpu_m4_drm_provider_object *const *)first;
	const struct gpu_m4_drm_provider_object *b = *(struct gpu_m4_drm_provider_object *const *)second;
	unsigned long x = (unsigned long)a->backing.identity, y = (unsigned long)b->backing.identity;

	return x < y ? -1 : x > y;
}

static int gpu_m4_drm_provider_version_index(struct gpu_m4_drm_provider_version *v)
{
	u32 sorted = min(v->objects_sorted, v->object_count), n = v->object_count;
	struct gpu_m4_drm_provider_object **merged = NULL;

	if (v->objects_indexed)
		return 0;
	if (sorted && sorted < n) {
		merged = kvcalloc(n, sizeof(*merged), GFP_KERNEL);
		if (!merged)
			return -ENOMEM;
	}
	sort(v->objects + sorted, n - sorted, sizeof(*v->objects),
	     gpu_m4_drm_provider_object_compare, NULL);
	if (merged) {
		for (u32 i = 0, j = sorted, k = 0; k < n; k++)
			merged[k] = j == n || (i < sorted &&
				gpu_m4_drm_provider_object_compare(&v->objects[i], &v->objects[j]) <= 0) ?
				v->objects[i++] : v->objects[j++];
		memcpy(v->objects, merged, array_size(n, sizeof(*merged)));
		kvfree(merged);
	}
	for (u32 i = 1; i < v->object_count; i++)
		if (v->objects[i - 1]->backing.identity == v->objects[i]->backing.identity)
			return -EPROTO;
	v->objects_indexed = true;
	return 0;
}

static int gpu_m4_drm_provider_version_prune(struct gpu_m4_drm_provider_version *v)
{
	u32 kept = 0;
	int ret = gpu_m4_drm_provider_version_index(v);

	if (ret)
		return ret;
	memset(v->object_used, 0, v->object_count * sizeof(*v->object_used));
	/* Each immutable identity resolves through the same sorted owned inventory
	 * as the PA oracle. Native cache-only/private objects must survive pruning.
	 */
	for (u32 i = 0; i < v->ledger.count + v->native_private_count; i++) {
		const void *identity = i < v->ledger.count ? v->ledger.leaf[i].identity :
			gpu_m4_drm_bo_pin_object(v->native_private[i - v->ledger.count].pin);
		u32 first = 0, end = v->object_count;

		while (first < end) {
			u32 middle = first + (end - first) / 2;
			unsigned long value = (unsigned long)v->objects[middle]->backing.identity;

			if (value < (unsigned long)identity)
				first = middle + 1;
			else
				end = middle;
		}
		if (first == v->object_count || v->objects[first]->backing.identity != identity)
			return -EPROTO;
		v->object_used[first] = true;
	}
	for (u32 i = 0; i < v->object_count; i++)
		if (v->object_used[i])
			v->objects[kept++] = v->objects[i];
		else
			gpu_m4_drm_provider_object_put_locked(v->objects[i]);
	v->object_count = kept; /* Stable compaction preserves the sorted index. */
	return 0;
}

static struct gpu_m4_drm_bo_pin *gpu_m4_drm_provider_lookup(void *cookie,
							const void *identity)
{
	struct gpu_m4_drm_provider_object *o = gpu_m4_drm_provider_version_object(cookie, identity);

	return o ? o->pin : NULL;
}

static int gpu_m4_drm_provider_page_valid(void *cookie, const void *identity,
					 u64 offset, u64 pa)
{
	struct gpu_m4_drm_provider_version *v = cookie;
	struct gpu_m4_drm_provider_object *o = gpu_m4_drm_provider_version_object(v, identity);
	u64 actual;
	int ret;

	if (!o || o->vm != v->vm || o->backing.identity != identity ||
	    !gpu_m4_drm_aligned(offset) || offset >= o->backing.size ||
	    o->backing.pa[offset / GPU_M4_DRM_PAGE_SIZE] != pa)
		return -EPERM;
	ret = gpu_m4_drm_bo_pin_page(o->pin, offset, &actual);
	if (ret || actual != pa)
		return ret ? ret : -EPERM;
	/* Registered owner, bounds and exact cached PA are proven above. An
	 * unchanged held pin uses its admission plus the live session/error
	 * gate instead of repeating the RAM/exclusion oracle per leaf.
	 */
	return gpu_m4_drm_provider_object_oracle(o, offset, pa);
}

int gpu_m4_drm_provider_version_get_locked(struct gpu_m4_drm_provider_version *v)
{
	if (!v)
		return -EINVAL;
	lockdep_assert_held(v->vm->provider->lock);
	return refcount_inc_not_zero(&v->refs) ? 0 : -ENOENT;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_get_locked);

int gpu_m4_drm_provider_version_put_locked(struct gpu_m4_drm_provider_version *v)
{
	lockdep_assert_held(v->vm->provider->lock);
	if (refcount_read(&v->refs) == 1 && (v->protected || v->working))
		return -EBUSY;
	if (refcount_dec_and_test(&v->refs))
		list_add_tail(&v->garbage, &v->vm->provider->garbage_versions);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_put_locked);

/* storage, when given, is a detached leaf array of at least max(capacity, 1)
 * entries that this version adopts ONLY on success; failure leaves it to the
 * caller. The charge covers the retained index of a full-capacity ledger.
 */
static struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_version_alloc_storage(
	struct gpu_m4_drm_provider_vm *vm, u32 capacity, u32 objects, u32 private_count,
	struct gpu_m4_drm_vm_leaf *storage)
{
	struct gpu_m4_drm_provider_version *v;
	struct gpu_m4_drm_provider *p = vm->provider;
	u64 bytes = sizeof(*v) + (u64)max(capacity, 1U) * sizeof(*v->ledger.leaf) +
		(u64)max(capacity, 1U) * 3 * sizeof(*v->index.va) +
		(u64)max(objects, 1U) * sizeof(*v->objects) +
		(u64)max(objects, 1U) * sizeof(*v->object_used) +
		(u64)private_count * (sizeof(*v->private) + sizeof(*v->native_private) +
				     sizeof(*v->private_leases));
	int ret;

	ret = gpu_m4_drm_provider_charge(p, bytes);
	if (ret)
		return ERR_PTR(ret);
	v = kzalloc_obj(*v);
	if (!v) {
		p->bytes -= bytes;
		return ERR_PTR(-ENOMEM);
	}
	v->ledger.leaf = storage ?: kvcalloc(max(capacity, 1U), sizeof(*v->ledger.leaf), GFP_KERNEL);
	v->objects = kcalloc(max(objects, 1U), sizeof(*v->objects), GFP_KERNEL);
	v->object_used = kcalloc(max(objects, 1U), sizeof(*v->object_used), GFP_KERNEL);
	v->private = kvcalloc(private_count, sizeof(*v->private), GFP_KERNEL);
	v->native_private = kvcalloc(private_count, sizeof(*v->native_private), GFP_KERNEL);
	v->private_leases = kcalloc(private_count, sizeof(*v->private_leases), GFP_KERNEL);
	if (!v->ledger.leaf || !v->objects || !v->object_used || !v->private ||
	    !v->native_private || !v->private_leases) {
		if (v->ledger.leaf != storage)
			kvfree(v->ledger.leaf);
		kfree(v->objects);
		kfree(v->object_used);
		kvfree(v->private);
		kvfree(v->native_private);
		kfree(v->private_leases);
		kfree(v);
		p->bytes -= bytes;
		return ERR_PTR(-ENOMEM);
	}
	v->vm = vm;
	v->serial = ++p->version_serial;
	v->metadata_bytes = bytes;
	v->private_count = private_count;
	v->base_private_count = private_count;
	v->private[0] = (struct gpu_m4_drm_uat_private) {
		.identity = &vm->context, .cookie = &vm->context,
		.get = gpu_m4_drm_provider_context_get, .put = gpu_m4_drm_provider_context_put,
		.page = gpu_m4_drm_provider_context_page, .va = GPU_M4_DRM_UAT_CONTEXT_START,
		.size = GPU_M4_DRM_UAT_CONTEXT_SIZE,
		.access = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE,
		.cpu_policy = GPU_M4_DRM_CPU_NC,
	};
	v->ledger = (struct gpu_m4_drm_vm_ledger) {
		.leaf = v->ledger.leaf, .capacity = max(capacity, 1U),
		.low = p->config.vm_start, .high = p->config.vm_end,
		.kernel_start = vm->kernel_start, .kernel_end = vm->kernel_end,
		.owner = v, .validate_page = gpu_m4_drm_provider_page_valid,
	};
	refcount_set(&v->refs, 1);
	INIT_LIST_HEAD(&v->garbage);
	gpu_m4_drm_provider_vm_get_locked(vm);
	gpu_m4_drm_provider_version_link(vm, v);
	return v;
}

static struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_version_alloc(
	struct gpu_m4_drm_provider_vm *vm, u32 capacity, u32 objects, u32 private_count)
{
	return gpu_m4_drm_provider_version_alloc_storage(vm, capacity, objects, private_count, NULL);
}

/* Count-paired retained index of exactly v->ledger, or none. */
static bool gpu_m4_drm_provider_version_indexed(const struct gpu_m4_drm_provider_version *v)
{
	return gpu_m4_drm_vm_ledger_index_valid(&v->ledger, &v->index);
}

/* One private mapping's VA extent; end is the running maximum once sorted. */
struct gpu_m4_drm_provider_extent {
	u64 va, end;
};

static int gpu_m4_drm_provider_extent_compare(const void *first, const void *second)
{
	const struct gpu_m4_drm_provider_extent *a = first, *b = second;

	return a->va < b->va ? -1 : a->va > b->va;
}

/*
 * VA order of count extents (scratch: extent[count, 2 * count)). An ordered
 * tail, such as a composed version's VA-ordered baseline members, is kept:
 * only the head before it is sorted, then the two are merged once.
 */
static void gpu_m4_drm_provider_extent_order(struct gpu_m4_drm_provider_extent *extent,
	u32 count)
{
	struct gpu_m4_drm_provider_extent *merged = extent + count;
	u32 tail = count ? count - 1 : 0, i, j, k;

	while (tail && extent[tail - 1].va <= extent[tail].va)
		tail--;
	if (!tail)
		return;
	sort(extent, tail, sizeof(*extent), gpu_m4_drm_provider_extent_compare, NULL);
	for (i = 0, j = tail, k = 0; k < count; k++)
		merged[k] = j == count || (i < tail && extent[i].va <= extent[j].va) ?
			extent[i++] : extent[j++];
	memcpy(extent, merged, array_size(count, sizeof(*extent)));
}

/* Does any mapping start below `below` and end above `above`? Extents are
 * sorted by VA with a running maximum end: one search, exact predicate.
 */
static bool gpu_m4_drm_provider_extent_any(const struct gpu_m4_drm_provider_extent *e,
	u32 count, u64 below, u64 above)
{
	u32 first = 0, end = count;

	while (first < end) {
		u32 middle = first + (end - first) / 2;

		if (e[middle].va < below)
			first = middle + 1;
		else
			end = middle;
	}
	return first && e[first - 1].end > above;
}

static int gpu_m4_drm_provider_version_shape(struct gpu_m4_drm_provider_version *v,
	u64 *leaf_count)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;
	struct gpu_m4_drm_provider_extent *extent;
	u64 leaves = v->ledger.count;
	u32 fixed = 0;
	int ret;

	/* Every input below (user count, generation, private mappings, held
	 * claims, sorted objects and the provider ceiling) is final before a
	 * version's first shape check and never changes afterwards.
	 */
	if (v->shape_leaves) {
		*leaf_count = v->shape_leaves;
		return 0;
	}
	ret = gpu_m4_drm_provider_version_index(v);
	if (ret)
		return ret;
	if (!v->generation || !v->private_count)
		return -EINVAL;
	for (u32 i = 0; i < v->private_count; i++) {
		const struct gpu_m4_drm_uat_private *m = &v->private[i];

		if (!m->identity || !m->cookie || !m->get || !m->put || !m->page ||
		    !m->size || !gpu_m4_drm_aligned(m->va) ||
		    !gpu_m4_drm_aligned(m->offset) || !gpu_m4_drm_aligned(m->size) ||
		    m->size > U64_MAX - m->offset || !gpu_m4_drm_vm_leaf_attrs(m->access) ||
		    (m->cpu_policy != GPU_M4_DRM_CPU_NC &&
		     m->cpu_policy != GPU_M4_DRM_CPU_WRITEBACK))
			return -EINVAL;
		if (m->va == GPU_M4_DRM_UAT_CONTEXT_START) {
			if (m->size != GPU_M4_DRM_UAT_CONTEXT_SIZE || m->offset ||
			    m->access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE))
				return -EINVAL;
			fixed++;
		} else if (!gpu_m4_drm_range(m->va, m->size,
			GPU_M4_DRM_UAT_PRIVATE_START, GPU_M4_DRM_UAT_PRIVATE_END)) {
			return -EINVAL;
		}
		leaves += m->size / GPU_M4_DRM_PAGE_SIZE;
		if (leaves > (u64)p->config.max_leaves + 4)
			return -ENOSPC;
	}
	if (fixed != 1)
		return -EINVAL;
	if (!v->private_lease_count)
		goto shaped;
	/* No mapping may overlap a held claim's guard: [start, va) or
	 * [va + size, end). The exact pairwise predicate, in claim order, over
	 * the mappings ordered once by VA instead of rescanned for every claim.
	 */
	extent = kvcalloc(2 * (u64)v->private_count, sizeof(*extent), GFP_KERNEL);
	if (!extent)
		return -ENOMEM;
	for (u32 i = 0; i < v->private_count; i++) {
		const struct gpu_m4_drm_uat_private *m = &v->private[i];

		if (m->size > U64_MAX - m->va) {
			kvfree(extent);
			return -EINVAL;
		}
		extent[i] = (struct gpu_m4_drm_provider_extent) { m->va, m->va + m->size };
	}
	gpu_m4_drm_provider_extent_order(extent, v->private_count);
	for (u32 i = 1; i < v->private_count; i++)
		extent[i].end = max(extent[i].end, extent[i - 1].end);
	for (u32 i = 0; i < v->private_lease_count; i++) {
		const struct gpu_m4_drm_provider_private_lease *lease = v->private_leases[i];

		if (!lease || !refcount_read(&lease->refs) || lease->vm != v->vm) {
			ret = -EPERM;
			break;
		}
		if (gpu_m4_drm_provider_extent_any(extent, v->private_count, lease->va, lease->start) ||
		    gpu_m4_drm_provider_extent_any(extent, v->private_count, lease->end,
						    lease->va + lease->size)) {
			ret = -EADDRINUSE;
			break;
		}
	}
	kvfree(extent);
	if (ret)
		return ret;
shaped:
	v->shape_leaves = leaves;
	*leaf_count = leaves;
	return 0;
}

/* Identical physical-RAM admission to the UAT builder. Delaying tables must
 * not let a private callback commit MMIO, invalid or reserved backing first.
 */
static int gpu_m4_drm_provider_private_ram(u64 pa)
{
	if (!pa || (pa & ~GPU_M4_PA_MASK) || PAGE_SIZE > GPU_M4_DRM_PAGE_SIZE ||
	    region_intersects(pa, GPU_M4_DRM_PAGE_SIZE, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_INTERSECTS)
		return -EPERM;
	for (u64 offset = 0; offset < GPU_M4_DRM_PAGE_SIZE; offset += PAGE_SIZE) {
		unsigned long pfn = (pa + offset) >> PAGE_SHIFT;

		if (!pfn_valid(pfn) || PageReserved(pfn_to_page(pfn)))
			return -EPERM;
	}
	return 0;
}

/* Facts a detached planner proved about v's user ledger without the lock. */
struct gpu_m4_drm_provider_user_facts {
	u32 added_from;	/* leaves [0, added_from) are unchanged source leaves */
	u32 l1, l2;	/* distinct VA >> 36 and VA >> 25 prefixes of the user ledger */
};

static bool gpu_m4_drm_provider_leaf_equal(const struct gpu_m4_drm_vm_leaf *a,
					   const struct gpu_m4_drm_vm_leaf *b)
{
	return a->va == b->va && a->pa == b->pa && a->object_offset == b->object_offset &&
		a->identity == b->identity && a->access == b->access &&
		a->cpu_policy == b->cpu_policy;
}

static bool gpu_m4_drm_provider_private_same(const struct gpu_m4_drm_uat_private *a,
					     const struct gpu_m4_drm_uat_private *b)
{
	return a->identity == b->identity && a->cookie == b->cookie && a->get == b->get &&
		a->put == b->put && a->page == b->page && a->va == b->va &&
		a->offset == b->offset && a->size == b->size && a->access == b->access &&
		a->cpu_policy == b->cpu_policy;
}

/* Does any user leaf share this VA prefix (shift 36: L1, shift 25: L2)? */
static bool gpu_m4_drm_provider_user_prefix(const struct gpu_m4_drm_provider_version *v,
					    u64 prefix, u32 shift)
{
	struct gpu_m4_drm_vm_leaf key = { .va = prefix << shift };
	u32 at = gpu_m4_drm_vm_pos_lower_bound(v->ledger.leaf, v->index.va, v->index.count,
					       &key, GPU_M4_DRM_VM_KEY_VA);

	return at < v->index.count && v->ledger.leaf[v->index.va[at]].va >> shift == prefix;
}

/*
 * Commit-time proof for a graph-free logical version. Private callbacks are
 * acquired independently: snapshots retain native pins, NOT external READY
 * cookie wrappers. The holds survive every subsequent snapshot-only capture.
 *
 * The user ledger is proven by its retained index (or fully validated once
 * here). The private manifest is expanded, admitted and proven internally;
 * when it is identical to the source's already cross-checked manifest, only
 * the changed user leaves are checked against it. A source holding the exact
 * same private inventory (it and v both hold every owner, whose pages cannot
 * change while held) already proved exactly that manifest: its leaves and
 * sorted VA/PA/identity positions are copied, never resolved or sorted again.
 * Table demand counts the distinct prefixes of user plus private leaves
 * without a combined sort.
 */
static int gpu_m4_drm_provider_version_logical(struct gpu_m4_drm_provider_version *v,
	struct gpu_m4_drm_provider_version *source,
	const struct gpu_m4_drm_provider_user_facts *facts)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;
	struct gpu_m4_drm_vm_ledger_index pindex = { 0 };
	struct gpu_m4_drm_vm_leaf *manifest = NULL;
	u64 count, bytes, last1 = U64_MAX, last2 = U64_MAX;
	u32 at = 0, from = 0, users = v->ledger.count, pcount, l1, l2, tables;
	bool same = false;
	int ret = gpu_m4_drm_provider_version_shape(v, &count);

	if (ret)
		return ret;
	if (!gpu_m4_drm_provider_version_indexed(v)) {
		/* An unproven ledger receives the complete original validation. */
		gpu_m4_drm_vm_ledger_index_free(&v->index);
		ret = gpu_m4_drm_vm_ledger_index_create(&v->ledger, &v->index);
		if (ret)
			return ret;
		facts = NULL;
	}
	for (u32 i = 0; i < v->private_count; i++) {
		const struct gpu_m4_drm_uat_private *m = &v->private[i];

		ret = m->get(m->cookie, v->vm->frontend, m->identity);
		if (ret)
			return ret < 0 ? ret : -EPROTO;
		v->private_held++;
	}
	pcount = count - users;
	bytes = (u64)max(pcount, 1U) * (sizeof(*manifest) + 3 * sizeof(*pindex.va));
	ret = gpu_m4_drm_provider_charge(p, bytes);
	if (ret)
		return ret;
	manifest = kvcalloc(max(pcount, 1U), sizeof(*manifest), GFP_KERNEL);
	if (!manifest || gpu_m4_drm_vm_ledger_index_alloc(&pindex, pcount)) {
		ret = -ENOMEM;
		goto out;
	}
	if (source && source->manifest_checked && source->manifest_count == pcount &&
	    source->manifest_index.count == pcount &&
	    source->private_held == source->private_count &&
	    source->private_count == v->private_count) {
		same = true;
		for (u32 i = 0; same && i < v->private_count; i++)
			same = gpu_m4_drm_provider_private_same(&v->private[i], &source->private[i]);
	}
	if (same) {
		memcpy(manifest, source->manifest, array_size(pcount, sizeof(*manifest)));
		if (pcount)
			memcpy(pindex.va, source->manifest_index.va,
			       array_size(3 * (u64)pcount, sizeof(*pindex.va)));
		/* The exact RAM admission is still made now, as resolving would. */
		for (u32 i = 0; i < pcount; i++) {
			ret = gpu_m4_drm_provider_private_ram(manifest[i].pa);
			if (ret)
				goto out;
		}
		if (facts && facts->added_from <= users)
			from = facts->added_from;
		goto checked;
	}
	for (u32 i = 0; i < v->private_count; i++) {
		const struct gpu_m4_drm_uat_private *m = &v->private[i];

		for (u64 offset = 0; offset < m->size; offset += GPU_M4_DRM_PAGE_SIZE) {
			struct gpu_m4_drm_vm_leaf *x = &manifest[at];

			*x = (struct gpu_m4_drm_vm_leaf) {
				.va = m->va + offset, .identity = m->identity,
				.object_offset = m->offset + offset,
				.access = m->access, .cpu_policy = m->cpu_policy,
			};
			ret = m->page(m->cookie, x->object_offset, &x->pa);
			if (ret)
				goto out;
			ret = gpu_m4_drm_provider_private_ram(x->pa);
			if (ret)
				goto out;
			pindex.va[at] = pindex.pa[at] = pindex.identity[at] = at;
			at++;
		}
	}
	/* Unchanged user leaves were proven against an identical manifest. */
	if (facts && source && source->manifest_checked && source->manifest_count == pcount &&
	    facts->added_from <= users) {
		from = facts->added_from;
		for (u32 i = 0; i < pcount && from; i++)
			if (!gpu_m4_drm_provider_leaf_equal(&manifest[i], &source->manifest[i]))
				from = 0;
	}
	gpu_m4_drm_vm_pos_sort(manifest, pindex.va, pcount, GPU_M4_DRM_VM_KEY_VA);
	for (u32 i = 1; i < pcount; i++)
		if (manifest[pindex.va[i - 1]].va == manifest[pindex.va[i]].va) {
			ret = -EADDRINUSE;
			goto out;
		}
checked:
	for (u32 i = from; i < users; i++)
		if (gpu_m4_drm_vm_ledger_index_find(manifest, &pindex, v->ledger.leaf[i].va) != ~0U) {
			ret = -EADDRINUSE;
			goto out;
		}
	if (facts) {
		l1 = facts->l1;
		l2 = facts->l2;
	} else {
		gpu_m4_drm_vm_ledger_prefixes(v->ledger.leaf, v->index.va, users, &l1, &l2);
	}
	tables = 1 + l1 + l2;
	for (u32 i = 0; i < pcount; i++) {
		u64 va = manifest[pindex.va[i]].va;

		if (va >> 36 != last1) {
			last1 = va >> 36;
			tables += !gpu_m4_drm_provider_user_prefix(v, last1, 36);
		}
		if (va >> 25 != last2) {
			last2 = va >> 25;
			tables += !gpu_m4_drm_provider_user_prefix(v, last2, 25);
		}
	}
	if (tables > p->config.max_tables) {
		ret = -ENOSPC;
		goto out;
	}
	if (!same) {
		gpu_m4_drm_vm_pos_sort(manifest, pindex.pa, pcount, GPU_M4_DRM_VM_KEY_PA);
		gpu_m4_drm_vm_pos_sort(manifest, pindex.identity, pcount, GPU_M4_DRM_VM_KEY_IDENTITY);
	}
	for (u32 i = 1; !same && i < pcount; i++) {
		ret = gpu_m4_drm_vm_leaf_alias_validate(&manifest[pindex.pa[i - 1]],
							&manifest[pindex.pa[i]]);
		if (!ret)
			ret = gpu_m4_drm_vm_leaf_alias_validate(&manifest[pindex.identity[i - 1]],
								&manifest[pindex.identity[i]]);
		if (ret)
			goto out;
	}
	for (u32 i = from; i < users; i++) {
		ret = gpu_m4_drm_vm_ledger_index_alias(manifest, &pindex, &v->ledger.leaf[i]);
		if (ret)
			goto out;
	}
	kvfree(v->manifest);
	gpu_m4_drm_vm_ledger_index_free(&v->manifest_index);
	v->manifest = manifest;
	v->manifest_index = pindex;
	v->manifest_count = pcount;
	v->manifest_checked = true;
	v->metadata_bytes += bytes;
	manifest = NULL;
	pindex = (struct gpu_m4_drm_vm_ledger_index) { 0 };
	bytes = 0;
	v->state = GPU_M4_PROVIDER_VERSION_LOGICAL;
out:
	gpu_m4_drm_vm_ledger_index_free(&pindex);
	kvfree(manifest);
	p->bytes -= bytes;
	return ret;
}

static int gpu_m4_drm_provider_version_snapshot(struct gpu_m4_drm_provider_version *v,
	struct gpu_m4_drm_provider_version *source)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;
	struct gpu_m4_drm_vm_owner *snapshot;
	u32 source_accesses = 0, extra_count = v->native_private_count;
	const struct gpu_m4_drm_cache_access *extra = v->native_private;
	u64 leaves, metadata, actual;
	int ret;

	if (v->state >= GPU_M4_PROVIDER_VERSION_SNAPSHOT)
		return 0;
	if (v->snapshot || v->lineage || v->uat)
		return -ESTALE;
	ret = gpu_m4_drm_provider_version_shape(v, &leaves);
	if (ret)
		return ret;

	if (source) {
		if (source->state < GPU_M4_PROVIDER_VERSION_SNAPSHOT ||
		    source->native_private_count > extra_count)
			return -EPROTO;
		extra += source->native_private_count;
		extra_count -= source->native_private_count;
		gpu_m4_drm_vm_owner_cache_access(source->snapshot, &source_accesses);
	} else {
		source_accesses = v->ledger.count;
	}
	if (extra_count > U32_MAX - source_accesses)
		return -E2BIG;

	/* Snapshot and graph charges are independent. No table allowance or table
	 * allocation belongs to an ordinary acceptance's throwaway base snapshot.
	 */
	metadata = 0x400 + 2 * ((u64)max(v->ledger.count, 1U) *
		(sizeof(*v->ledger.leaf) + 3 * sizeof(*v->index.va)) +
		(u64)max(source_accesses + extra_count, 1U) *
		(sizeof(struct gpu_m4_drm_cache_access) + sizeof(struct gpu_m4_drm_resv_access) +
		 sizeof(u32)));
	ret = gpu_m4_drm_provider_charge(p, metadata);
	if (ret)
		return ret;
	/* A clone shares the source's immutable leaves/indexes; a new base copies
	 * this version's retained index instead of re-proving every alias.
	 */
	if (source)
		snapshot = gpu_m4_drm_vm_owner_clone_with_extras(source->snapshot, extra, extra_count);
	else
		snapshot = gpu_m4_drm_vm_owner_create_indexed(v->vm->frontend, p->lock, &v->ledger,
			gpu_m4_drm_provider_version_indexed(v) ? &v->index : NULL,
			v->generation, v, gpu_m4_drm_provider_lookup, extra, extra_count);
	if (IS_ERR(snapshot)) {
		p->bytes -= metadata;
		return PTR_ERR(snapshot);
	}
	/* The bound above admitted construction. Retain only what this version
	 * actually keeps alive: its wrapper and set, the shared immutable map
	 * (once, also covering a lineage map that may outlive its source), and
	 * a distinct lineage set. Shared maps are never charged twice.
	 */
	actual = 0x100 + gpu_m4_drm_vm_owner_bytes(snapshot, true);
	if (source && source->lineage)
		actual += gpu_m4_drm_vm_owner_bytes(source->lineage, false);
	if (actual < metadata) {
		p->bytes -= metadata - actual;
		metadata = actual;
	}
	v->snapshot = snapshot;
	v->metadata_bytes += metadata;
	v->lineage = source ? source->lineage : snapshot;
	ret = gpu_m4_drm_vm_owner_get_unpublished(v->lineage);
	if (ret) {
		v->lineage = NULL;
		return ret;
	}
	v->state = GPU_M4_PROVIDER_VERSION_SNAPSHOT;
	return 0;
}

/*
 * Copy-on-write graph source for v: an already constructed immutable GRAPH of
 * the same VM, user generation and snapshot lineage (one shared immutable
 * user map) whose private inventory is a prefix of v's. The builder rechecks
 * the exact prefix, VM, generation and user count; anything else is a full
 * build. Its held constructed identity replaces another full verify.
 */
static struct gpu_m4_drm_uat *gpu_m4_drm_provider_graph_source(
	struct gpu_m4_drm_provider_version *v, struct gpu_m4_drm_provider_version *source)
{
	if (!source || source == v || source->vm != v->vm ||
	    source->state != GPU_M4_PROVIDER_VERSION_GRAPH || IS_ERR_OR_NULL(source->uat) ||
	    !refcount_read(&source->refs) || !v->lineage || source->lineage != v->lineage ||
	    source->generation != v->generation || source->private_count > v->private_count ||
	    gpu_m4_drm_uat_check(source->uat))
		return NULL;
	return source->uat;
}

/* Charge metadata and the builder's maximum permitted actual table allocation
 * (refunded to the real use), so no temporary graph can exceed the ceiling.
 */
static int gpu_m4_drm_provider_graph_reserve(struct gpu_m4_drm_provider_version *v,
	struct gpu_m4_drm_provider_graph_garbage **rollback, u64 *metadata, u64 *reserved,
	u32 *available)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;
	u64 leaves;
	int ret;

	if (v->state != GPU_M4_PROVIDER_VERSION_SNAPSHOT || v->uat ||
	    v->protected || v->working || v->binding_asid)
		return -EINVAL;
	ret = gpu_m4_drm_provider_version_shape(v, &leaves);
	if (ret)
		return ret;
	/* A rebase transiently also holds a retained base's manifest block. */
	leaves += max(v->vm->graph_cache ? gpu_m4_drm_uat_leaves(v->vm->graph_cache->uat) : 0,
		      v->vm->graph_spare ? gpu_m4_drm_uat_leaves(v->vm->graph_spare->uat) : 0);
	*metadata = 0x200 + gpu_m4_drm_uat_metadata_bytes(leaves, v->private_count,
							  p->config.max_tables);
	ret = gpu_m4_drm_provider_charge(p, *metadata + sizeof(**rollback));
	if (ret)
		return ret;
	*rollback = kzalloc_obj(**rollback);
	if (!*rollback) {
		p->bytes -= *metadata + sizeof(**rollback);
		return -ENOMEM;
	}
	/* A removal-only replacement may also reserve within its allowance,
	 * bounded by what remains of the provider-wide removal reserve.
	 */
	*available = min_t(u64, p->config.max_tables,
		max(gpu_m4_drm_provider_headroom(p),
		    min(p->replacing, gpu_m4_drm_provider_reserve_headroom(p))) /
		GPU_M4_DRM_PAGE_SIZE);
	*reserved = (u64)*available * GPU_M4_DRM_PAGE_SIZE;
	if (!*available) {
		kfree(*rollback);
		p->bytes -= *metadata + sizeof(**rollback);
		p->pressure = true;
		return -ENOSPC;
	}
	p->replacing -= min(*reserved, p->replacing);
	p->bytes += *reserved;
	return 0;
}

/* Return one borrow (lock held, no callback): drop the append source ref; a
 * detached cache carrier is handed to the reaper once its last borrower is
 * gone, so its pages' charge outlives every graph naming them.
 */
static void gpu_m4_drm_provider_borrow_put_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_graph_borrow *b)
{
	lockdep_assert_held(p->lock);
	if (b->version)
		gpu_m4_drm_provider_version_put_locked(b->version);
	if (b->cache && !--b->cache->borrows && b->cache->detached)
		list_add_tail(&b->cache->link, &p->garbage_graphs);
	*b = (struct gpu_m4_drm_provider_graph_borrow) { 0 };
}

/* Builder rollback retains every acquired callback owner and partial table
 * until reap can release them outside the shared mutex. A failed derived
 * candidate keeps its own fresh pages charged, and keeps borrowing the charge
 * owner of the shared base pages it still names until the reaper has released
 * the partial graph.
 */
static void gpu_m4_drm_provider_graph_fail(struct gpu_m4_drm_provider_version *v,
	struct gpu_m4_drm_provider_graph_garbage *rollback, struct gpu_m4_drm_uat *garbage,
	u64 metadata, u64 reserved, struct gpu_m4_drm_provider_graph_borrow *borrow)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;

	if (garbage) {
		u64 owned = gpu_m4_drm_uat_owned_bytes(garbage);

		rollback->version = v;
		rollback->uat = garbage;
		rollback->bytes = metadata + sizeof(*rollback) + owned;
		rollback->base_version = borrow->version;
		rollback->base_cache = borrow->cache;
		*borrow = (struct gpu_m4_drm_provider_graph_borrow) { 0 };
		gpu_m4_drm_provider_version_get_locked(v);
		list_add_tail(&rollback->link, &p->garbage_graphs);
		p->bytes -= reserved - owned;
	} else {
		kfree(rollback);
		p->bytes -= reserved + metadata + sizeof(*rollback);
		gpu_m4_drm_provider_borrow_put_locked(p, borrow);
	}
}

/*
 * Compact-retry handoff under the lock: the failed attempt's pages, node refs
 * and manifest blocks are dropped now (callback-free), then its base borrow;
 * only the snapshot/private-owner remainder goes to the outside-lock reaper,
 * charged exactly. The retry's reservation never pays for disposable pages.
 */
static void gpu_m4_drm_provider_graph_strip_fail(struct gpu_m4_drm_provider_version *v,
	struct gpu_m4_drm_provider_graph_garbage *rollback, struct gpu_m4_drm_uat *garbage,
	u64 metadata, u64 reserved, struct gpu_m4_drm_provider_graph_borrow *borrow)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;
	u64 remaining;

	if (gpu_m4_drm_uat_strip(garbage)) {
		gpu_m4_drm_provider_graph_fail(v, rollback, garbage, metadata, reserved, borrow);
		return;
	}
	gpu_m4_drm_provider_borrow_put_locked(p, borrow);
	remaining = gpu_m4_drm_uat_metadata_used(garbage);
	if (remaining > metadata)
		remaining = metadata;
	gpu_m4_drm_provider_graph_fail(v, rollback, garbage, remaining,
		reserved + metadata - remaining, borrow);
}

/* A constructed graph names at most `available` pages: charge all of them,
 * shared ones included, conservatively for this version's whole lifetime.
 * Only once that complete reachable-table charge is installed may the base
 * borrow be returned.
 */
static void gpu_m4_drm_provider_graph_install(struct gpu_m4_drm_provider_version *v,
	struct gpu_m4_drm_provider_graph_garbage *rollback, struct gpu_m4_drm_uat *uat,
	u64 metadata, u64 reserved, struct gpu_m4_drm_provider_graph_borrow *borrow)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;
	u64 used = gpu_m4_drm_uat_metadata_used(uat);

	/* Keep the actual reachable metadata charged, never more than reserved. */
	if (used < metadata) {
		p->bytes -= metadata - used;
		metadata = used;
	}
	/* A ticket of the SNAPSHOT state is republished with this GRAPH by the
	 * next locked capture, which checks it like every GRAPH capture.
	 */
	if (v->vm->capture == v)
		gpu_m4_drm_provider_vm_withdraw_locked(v->vm);
	v->uat = uat;
	v->table_bytes = gpu_m4_drm_uat_table_bytes(uat);
	v->metadata_bytes += metadata;
	v->state = GPU_M4_PROVIDER_VERSION_GRAPH;
	/* Installed only with no provider fault: the graph's construction proof
	 * (fresh pages checked word by word, shared nodes by their creators) is
	 * current. Ordinary refreshes consume it instead of rescanning.
	 */
	v->refreshed_revision = p->fault_revision;
	p->bytes -= reserved - v->table_bytes;
	p->bytes -= sizeof(*rollback);
	kfree(rollback);
	gpu_m4_drm_provider_borrow_put_locked(p, borrow);
}

/* Hand a replaced/cleared construction cache to the reaper (outside the lock
 * it releases the image, then refunds exactly its own charge).
 */
static void gpu_m4_drm_provider_graph_slot_drop_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_graph_garbage **slot)
{
	struct gpu_m4_drm_provider_graph_garbage *g = *slot;

	lockdep_assert_held(p->lock);
	if (!g)
		return;
	*slot = NULL;
	/* Undiscoverable now; refunded only after its last borrower. */
	g->detached = true;
	if (!g->borrows)
		list_add_tail(&g->link, &p->garbage_graphs);
}

static void gpu_m4_drm_provider_graph_cache_drop_locked(struct gpu_m4_drm_provider_vm *vm)
{
	gpu_m4_drm_provider_graph_slot_drop_locked(vm->provider, &vm->graph_cache);
	gpu_m4_drm_provider_graph_slot_drop_locked(vm->provider, &vm->graph_spare);
	vm->graph_cache_generation = vm->graph_cache_revision = 0;
	vm->graph_spare_generation = vm->graph_spare_revision = 0;
}

/*
 * After a successful materialization: retain this graph as the VM's user-base
 * construction cache when it is exactly the common private baseline (same
 * private inventory as the active version, no native extension) and
 * rebase-ready. A newer key becomes the cache (the old one the spare); an
 * older key not already retained becomes the spare, so an older accepted
 * capture's later children share it rather than rebase again each time.
 * Failure only skips it.
 */
static void gpu_m4_drm_provider_graph_cache_locked(struct gpu_m4_drm_provider_version *v)
{
	struct gpu_m4_drm_provider_vm *vm = v->vm;
	struct gpu_m4_drm_provider *p = vm->provider;
	struct gpu_m4_drm_provider_graph_garbage *g;
	struct gpu_m4_drm_uat *image;
	u64 bytes;

	u64 revision = v->baseline ? v->baseline->revision : 0;
	bool newer = !vm->graph_cache || v->generation > vm->graph_cache_generation ||
		(v->generation == vm->graph_cache_generation && revision > vm->graph_cache_revision);
	u32 base;

	lockdep_assert_held(p->lock);
	/* Keyed by held user map (generation) AND private membership revision;
	 * eligible only as exactly the common private baseline, optionally plus
	 * exactly its held native baseline's mapping-only members. A child's job
	 * accesses (native_private[]) never make the cache retain its access set.
	 */
	if (vm->closed || p->error || v->state != GPU_M4_PROVIDER_VERSION_GRAPH ||
	    v->native_private_count || !vm->active ||
	    (!newer && ((v->generation == vm->graph_cache_generation &&
			 revision == vm->graph_cache_revision) ||
			(vm->graph_spare && v->generation == vm->graph_spare_generation &&
			 revision == vm->graph_spare_revision))) ||
	    !gpu_m4_drm_uat_rebase_ready(v->uat))
		return;
	base = vm->active->private_count;
	if (v->base_private_count != base ||
	    v->private_count != base + (v->baseline ? v->baseline->count : 0))
		return;
	for (u32 i = 0; i < base; i++)
		if (!gpu_m4_drm_provider_private_same(&v->private[i], &vm->active->private[i]))
			return;
	g = kzalloc_obj(*g);
	if (!g)
		return;
	image = gpu_m4_drm_uat_image(v->uat);
	if (IS_ERR(image)) {
		kfree(g);
		return;
	}
	bytes = sizeof(*g) + gpu_m4_drm_uat_table_bytes(image) +
		gpu_m4_drm_uat_metadata_used(image);
	/* Optional: never a capacity refusal/report for a cache, and never
	 * retained while the provider is under charge pressure.
	 */
	if (p->pressure || bytes > gpu_m4_drm_provider_headroom(p)) {
		gpu_m4_drm_uat_release(image); /* callback-free, cannot be busy */
		kfree(g);
		return;
	}
	p->bytes += bytes;
	*g = (struct gpu_m4_drm_provider_graph_garbage) { .uat = image, .bytes = bytes };
	INIT_LIST_HEAD(&g->link);
	gpu_m4_drm_provider_graph_slot_drop_locked(p, &vm->graph_spare);
	if (newer) {
		vm->graph_spare = vm->graph_cache;
		vm->graph_spare_generation = vm->graph_cache_generation;
		vm->graph_spare_revision = vm->graph_cache_revision;
		vm->graph_cache = g;
		vm->graph_cache_generation = v->generation;
		vm->graph_cache_revision = revision;
	} else {
		vm->graph_spare = g;
		vm->graph_spare_generation = v->generation;
		vm->graph_spare_revision = revision;
	}
}

/* The retained image (cache or spare) of exactly v's key, or NULL. */
static struct gpu_m4_drm_provider_graph_garbage *gpu_m4_drm_provider_graph_exact(
	const struct gpu_m4_drm_provider_version *v, u64 revision)
{
	const struct gpu_m4_drm_provider_vm *vm = v->vm;

	if (vm->graph_cache && vm->graph_cache_generation == v->generation &&
	    vm->graph_cache_revision == revision)
		return vm->graph_cache;
	if (vm->graph_spare && vm->graph_spare_generation == v->generation &&
	    vm->graph_spare_revision == revision)
		return vm->graph_spare;
	return NULL;
}

/* Rebase base for v: the retained image of the same private revision (only
 * the user delta), else of the same generation (only the private delta),
 * else the newest. Its revision is the kept proof's.
 */
static struct gpu_m4_drm_provider_graph_garbage *gpu_m4_drm_provider_graph_rebase_base(
	const struct gpu_m4_drm_provider_version *v, u64 revision, u64 *base_revision)
{
	const struct gpu_m4_drm_provider_vm *vm = v->vm;
	bool spare = vm->graph_spare && vm->graph_cache_revision != revision &&
		(vm->graph_spare_revision == revision ||
		 (vm->graph_cache_generation != v->generation &&
		  vm->graph_spare_generation == v->generation));

	*base_revision = spare ? vm->graph_spare_revision : vm->graph_cache_revision;
	return spare ? vm->graph_spare : vm->graph_cache;
}

/*
 * Which of v's private mappings name backing that the VM's cache image already
 * resolved and that stayed held ever since, so a rebase keeps the image's
 * leaves for them instead of resolving every page again (the builder still
 * requires exactly that descriptor as a run of the image's block, else
 * resolves; a kept run's pages are not even visited). The common
 * private baseline is fixed for the VM's open lifetime: initialized once and
 * held by every active version, and the cache dies at close. A native
 * baseline member is the same object in every baseline of this VM's single
 * revision chain from its admission revision on, each holding it before its
 * predecessor drops: kept iff admitted no later than the image's revision.
 * That is the share key's argument, per member instead of per revision.
 */
static void gpu_m4_drm_provider_graph_kept(const struct gpu_m4_drm_provider_version *v,
	u64 revision, bool *kept)
{
	const struct gpu_m4_drm_provider_baseline *b = v->baseline;
	u32 base = min(v->base_private_count, v->private_count);

	lockdep_assert_held(v->vm->provider->lock);
	for (u32 i = 0; i < base; i++)
		kept[i] = true;
	if (!b || !revision || v->private_count != base + b->count)
		return;
	for (u32 i = 0; i < b->count; i++)
		kept[base + i] = b->member[i].since && b->member[i].since <= revision;
}

enum gpu_m4_drm_provider_graph_kind {
	GPU_M4_PROVIDER_GRAPH_FULL,
	GPU_M4_PROVIDER_GRAPH_APPEND,
	GPU_M4_PROVIDER_GRAPH_REBASE,
	GPU_M4_PROVIDER_GRAPH_SHARE,
};

/*
 * Under the lock: the cheapest valid prepared construction of v's exact
 * captured target: a use wrapper of a retained image of exactly its key, else
 * same-lineage private append, else a cross-generation rebase over a retained
 * user-base image (a construction base only, held by real node/manifest
 * refs), else the complete build. A refused base
 * (nothing acquired) falls through, including a base whose retained table
 * inventory alone exceeds the allowance; any other error is v's own. compact
 * forces the complete build. The base's charge owner is borrowed BEFORE any
 * base node reference is taken.
 */
static struct gpu_m4_drm_uat *gpu_m4_drm_provider_graph_prepare(
	struct gpu_m4_drm_provider_version *v, struct gpu_m4_drm_provider_version *source,
	u32 available, bool compact, struct gpu_m4_drm_uat **garbage,
	enum gpu_m4_drm_provider_graph_kind *kind, struct gpu_m4_drm_provider_graph_borrow *borrow,
	bool *compactable)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;
	struct gpu_m4_drm_uat *base = compact ? NULL : gpu_m4_drm_provider_graph_source(v, source);
	struct gpu_m4_drm_uat *uat = ERR_PTR(-ESTALE);
	u32 leaves = p->config.max_leaves + 4;
	struct gpu_m4_drm_provider_graph_garbage *cache;
	u64 revision = v->baseline ? v->baseline->revision : 0, cached;

	*garbage = NULL;
	*borrow = (struct gpu_m4_drm_provider_graph_borrow) { 0 };
	/* The exact complete user-map + private-baseline image is materialized
	 * once; every later child gets a use wrapper sharing its nodes and root
	 * (adoption-eligible), never a zero-delta append copying a root.
	 */
	cache = compact ? NULL : gpu_m4_drm_provider_graph_exact(v, revision);
	if (cache) {
		*kind = GPU_M4_PROVIDER_GRAPH_SHARE;
		*compactable = false;
		borrow->cache = cache;
		cache->borrows++;
		uat = gpu_m4_drm_uat_share_prepare(cache->uat, v->snapshot, v->private,
			v->private_count, available, garbage);
		if (!IS_ERR(uat) || *garbage)
			return uat;
		gpu_m4_drm_provider_borrow_put_locked(p, borrow);
		uat = ERR_PTR(-ESTALE);
	}
	*kind = GPU_M4_PROVIDER_GRAPH_APPEND;
	cache = compact ? NULL : gpu_m4_drm_provider_graph_rebase_base(v, revision, &cached);
	/* A composed child with the cached private membership rebases only its
	 * user delta instead of appending every baseline member again.
	 */
	if (cache && v->baseline && cached == revision)
		base = NULL;
	/* Only an inherited inventory with retained empty branches can be
	 * smaller when rebuilt: never retry a genuine target-capacity refusal.
	 */
	*compactable = base && gpu_m4_drm_uat_compactable(base);
	if (base) {
		gpu_m4_drm_provider_version_get_locked(source);
		borrow->version = source;
		uat = gpu_m4_drm_uat_prepare_deferred(base, v->snapshot, v->private,
			v->private_count, available, leaves, garbage);
	}
	if (IS_ERR(uat) && !*garbage)
		gpu_m4_drm_provider_borrow_put_locked(p, borrow);
	if (IS_ERR(uat) && !*garbage && !compact &&
	    (PTR_ERR(uat) == -ESTALE || PTR_ERR(uat) == -ENOSPC) &&
	    cache && gpu_m4_drm_provider_version_indexed(v)) {
		struct gpu_m4_drm_uat_user user = {
			.leaf = v->ledger.leaf, .va = v->index.va, .count = v->ledger.count,
		};
		bool *kept = kcalloc(v->private_count, sizeof(*kept), GFP_KERNEL);

		if (!kept)
			return ERR_PTR(-ENOMEM);
		gpu_m4_drm_provider_graph_kept(v, cached, kept);
		*kind = GPU_M4_PROVIDER_GRAPH_REBASE;
		*compactable = true; /* deletions empty inherited branches */
		borrow->cache = cache;
		cache->borrows++;
		uat = gpu_m4_drm_uat_rebase_prepare(cache->uat, v->snapshot, &user,
			v->private, kept, v->private_count, available, leaves, garbage);
		kfree(kept);
		if (IS_ERR(uat) && !*garbage)
			gpu_m4_drm_provider_borrow_put_locked(p, borrow);
	}
	if (IS_ERR(uat) && !*garbage && (PTR_ERR(uat) == -ESTALE || PTR_ERR(uat) == -ENOSPC)) {
		*kind = GPU_M4_PROVIDER_GRAPH_FULL;
		*compactable = false;
		uat = gpu_m4_drm_uat_prepare_deferred(NULL, v->snapshot, v->private,
			v->private_count, available, leaves, garbage);
	}
	return uat;
}

/* Only a derived construction's table-inventory refusal gets one compact,
 * complete retry of the same captured target and limits.
 */
static bool gpu_m4_drm_provider_graph_compact_retry(enum gpu_m4_drm_provider_graph_kind kind,
	bool compactable, int error)
{
	return error == -ENOSPC && (kind == GPU_M4_PROVIDER_GRAPH_APPEND ||
		kind == GPU_M4_PROVIDER_GRAPH_REBASE) && compactable;
}

static int gpu_m4_drm_provider_version_graph_locked(struct gpu_m4_drm_provider_version *v,
	struct gpu_m4_drm_provider_version *source)
{
	enum gpu_m4_drm_provider_graph_kind kind;
	struct gpu_m4_drm_provider_graph_borrow borrow;
	struct gpu_m4_drm_provider *p;
	struct gpu_m4_drm_provider_graph_garbage *rollback;
	bool compactable;
	struct gpu_m4_drm_uat *uat, *garbage;
	u64 metadata, reserved;
	u32 available;
	int ret;

	if (IS_ERR_OR_NULL(v))
		return -EINVAL;
	p = v->vm->provider;
	lockdep_assert_held(p->lock);
	if (p->error)
		return p->error;
	/* Another reference to an immutable GRAPH needs only its constructed
	 * identity, never another full verify; bind transitions consume the
	 * construction proof and check live roots.
	 */
	if (v->state == GPU_M4_PROVIDER_VERSION_GRAPH)
		return gpu_m4_drm_uat_check(v->uat);
	for (u32 attempt = 0;; attempt++) {
		ret = gpu_m4_drm_provider_graph_reserve(v, &rollback, &metadata, &reserved,
							&available);
		if (ret)
			return ret;
		uat = gpu_m4_drm_provider_graph_prepare(v, source, available, attempt, &garbage,
							&kind, &borrow, &compactable);
		if (!IS_ERR(uat)) {
			ret = gpu_m4_drm_uat_construct(uat);
			if (!ret)
				break;
			garbage = uat;
			uat = ERR_PTR(ret);
		}
		/* A derived graph out of table inventory (inherited/retained empty
		 * branches) is rebuilt once compactly from scratch. The caller holds
		 * the mutex: the failed attempt's pages are dropped now and only its
		 * callback owners are handed to the outside-lock reaper.
		 */
		if (!attempt && garbage && !p->error &&
		    gpu_m4_drm_provider_graph_compact_retry(kind, compactable, PTR_ERR(uat))) {
			gpu_m4_drm_provider_graph_strip_fail(v, rollback, garbage, metadata,
							     reserved, &borrow);
			continue;
		}
		gpu_m4_drm_provider_graph_fail(v, rollback, garbage, metadata, reserved, &borrow);
		return PTR_ERR(uat);
	}
	gpu_m4_drm_provider_graph_install(v, rollback, uat, metadata, reserved, &borrow);
	gpu_m4_drm_provider_graph_cache_locked(v);
	return 0;
}

int gpu_m4_drm_provider_version_ensure_graph_locked(struct gpu_m4_drm_provider_version *v)
{
	return gpu_m4_drm_provider_version_graph_locked(v, NULL);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_ensure_graph_locked);

/*
 * Detached construction: the provider mutex covers only the short prepare
 * (state/charge checks, snapshot/private holds, private page resolution, base
 * borrow and node/manifest holds) and the commit. Delta planning, allocation,
 * alias proof, table writes, cleaning and the construction proof run without
 * it. The commit installs only into the unchanged SNAPSHOT-state target
 * version; a lost race or provider fault discards the detached graph through
 * the ordinary outside-lock garbage owner. A derived attempt out of table
 * inventory is released here, outside the mutex, before the one compact retry
 * reserves again and rechecks the target.
 */
int gpu_m4_drm_provider_version_ensure_graph(struct gpu_m4_drm_provider_version *v,
	struct gpu_m4_drm_provider_version *source)
{
	enum gpu_m4_drm_provider_graph_kind kind;
	struct gpu_m4_drm_provider_graph_borrow borrow;
	struct gpu_m4_drm_provider *p;
	struct gpu_m4_drm_provider_graph_garbage *rollback;
	bool compactable;
	struct gpu_m4_drm_uat *uat, *garbage;
	u64 metadata, reserved;
	u32 available;
	int ret;

	if (IS_ERR_OR_NULL(v))
		return -EINVAL;
	p = v->vm->provider;
	lockdep_assert_not_held(p->lock);
	mutex_lock(p->lock);
	for (u32 attempt = 0;; attempt++) {
		if (!refcount_read(&v->refs)) {
			ret = -ENOENT;
			break;
		}
		if (p->error) {
			ret = p->error;
			break;
		}
		if (v->state == GPU_M4_PROVIDER_VERSION_GRAPH) {
			ret = gpu_m4_drm_uat_check(v->uat);
			break;
		}
		ret = gpu_m4_drm_provider_graph_reserve(v, &rollback, &metadata, &reserved,
							&available);
		if (ret)
			break;
		uat = gpu_m4_drm_provider_graph_prepare(v, source, available, attempt, &garbage,
							&kind, &borrow, &compactable);
		if (IS_ERR(uat)) {
			gpu_m4_drm_provider_graph_fail(v, rollback, garbage, metadata, reserved,
						       &borrow);
			ret = PTR_ERR(uat);
			break;
		}
		/* The prepared graph holds every input; this ref pins v and its charge. */
		gpu_m4_drm_provider_version_get_locked(v);
		mutex_unlock(p->lock);
		ret = gpu_m4_drm_uat_construct(uat);
		if (!attempt && gpu_m4_drm_provider_graph_compact_retry(kind, compactable, ret) &&
		    !gpu_m4_drm_uat_release(uat)) {
			/* Released outside the mutex: only then refund this attempt's
			 * whole reservation and return its base borrow.
			 */
			mutex_lock(p->lock);
			p->bytes -= reserved + metadata + sizeof(*rollback);
			kfree(rollback);
			gpu_m4_drm_provider_borrow_put_locked(p, &borrow);
			gpu_m4_drm_provider_version_put_locked(v);
			continue; /* rechecks the retained target first */
		}
		mutex_lock(p->lock);
		if (!ret && !p->error && v->state == GPU_M4_PROVIDER_VERSION_SNAPSHOT && !v->uat &&
		    !v->protected && !v->working && !v->binding_asid) {
			gpu_m4_drm_provider_graph_install(v, rollback, uat, metadata, reserved,
							  &borrow);
			gpu_m4_drm_provider_graph_cache_locked(v);
			gpu_m4_drm_provider_version_put_locked(v);
			break;
		}
		if (!ret)
			ret = p->error ? p->error :
				v->state == GPU_M4_PROVIDER_VERSION_GRAPH ?
				gpu_m4_drm_uat_check(v->uat) : -ESTALE;
		gpu_m4_drm_provider_graph_fail(v, rollback, uat, metadata, reserved, &borrow);
		gpu_m4_drm_provider_version_put_locked(v); /* never the final protected drop */
		break;
	}
	mutex_unlock(p->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_ensure_graph);

/* A derived version shares source's graph branches when source has one. */
static int gpu_m4_drm_provider_version_build(struct gpu_m4_drm_provider_version *v,
	struct gpu_m4_drm_provider_version *source)
{
	int ret = gpu_m4_drm_provider_version_snapshot(v, source);

	return ret ?: gpu_m4_drm_provider_version_graph_locked(v, source);
}

/* Give a SNAPSHOT-state source its own complete graph once, so this and every
 * later extension of the same captured user generation derive from it instead
 * of rebuilding the whole VM. Failure other than a provider fault only means
 * this candidate uses the complete build; the source stays as it was.
 */
static int gpu_m4_drm_provider_graph_base_locked(struct gpu_m4_drm_provider_version *source)
{
	struct gpu_m4_drm_provider *p = source->vm->provider;
	int ret;

	if (source->state != GPU_M4_PROVIDER_VERSION_SNAPSHOT)
		return 0;
	ret = gpu_m4_drm_provider_version_graph_locked(source, NULL);
	return ret && p->error ? p->error : 0;
}

bool gpu_m4_drm_provider_version_same_user_locked(
	struct gpu_m4_drm_provider_version *first, struct gpu_m4_drm_provider_version *second)
{
	if (!first || !second || first->vm != second->vm)
		return false;
	lockdep_assert_held(first->vm->provider->lock);
	if (!first->lineage || first->lineage != second->lineage ||
	    first->generation != second->generation ||
	    first->ledger.count != second->ledger.count)
		return false;
	/* Both user ledgers are immutable once constructed: a complete equal
	 * comparison between these exact versions (never-reused serials) stands.
	 * The same leaf storage (a borrowed ledger, held by both) is equal.
	 */
	if (first->user_serial == second->serial || second->user_serial == first->serial)
		return true;
	for (u32 i = 0; first->ledger.leaf != second->ledger.leaf && i < first->ledger.count; i++) {
		const struct gpu_m4_drm_vm_leaf *a = &first->ledger.leaf[i];
		const struct gpu_m4_drm_vm_leaf *b = &second->ledger.leaf[i];

		if (a->va != b->va || a->pa != b->pa || a->identity != b->identity ||
		    a->object_offset != b->object_offset || a->access != b->access ||
		    a->cpu_policy != b->cpu_policy)
			return false;
	}
	first->user_serial = second->serial;
	return true;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_same_user_locked);

/* Private/native/object/lease inventory and generation, not the user leaves. */
static int gpu_m4_drm_provider_version_copy_meta(
	struct gpu_m4_drm_provider_version *v, struct gpu_m4_drm_provider_version *source)
{
	int ret;

	memcpy(v->private, source->private,
		array_size(source->private_count, sizeof(*v->private)));
	memcpy(v->native_private, source->native_private,
		array_size(source->native_private_count, sizeof(*v->native_private)));
	v->native_private_count = source->native_private_count;
	for (u32 i = 0; i < source->object_count; i++) {
		ret = gpu_m4_drm_provider_object_get_locked(source->objects[i]);
		if (ret)
			return ret;
		v->objects[v->object_count++] = source->objects[i];
	}
	if (source->objects_indexed && v->object_count == source->object_count)
		v->objects_sorted = v->object_count;
	for (u32 i = 0; i < source->private_lease_count; i++) {
		ret = gpu_m4_drm_provider_private_get_locked(source->private_leases[i]);
		if (ret)
			return ret;
		v->private_leases[v->private_lease_count++] = source->private_leases[i];
	}
	v->generation = source->generation;
	v->base_private_count = source->base_private_count;
	v->baseline_revision = source->baseline_revision;
	return 0;
}

/* Exact user-ledger copy keeps the source's proven positions: copy the index. */
static int gpu_m4_drm_provider_version_copy_private(
	struct gpu_m4_drm_provider_version *v, struct gpu_m4_drm_provider_version *source)
{
	int ret;

	v->ledger.count = source->ledger.count;
	memcpy(v->ledger.leaf, source->ledger.leaf,
		array_size(source->ledger.count, sizeof(*v->ledger.leaf)));
	gpu_m4_drm_vm_ledger_index_free(&v->index);
	if (gpu_m4_drm_provider_version_indexed(source)) {
		ret = gpu_m4_drm_vm_ledger_index_copy(&v->index, &source->index);
		if (ret)
			return ret;
	}
	return gpu_m4_drm_provider_version_copy_meta(v, source);
}

static struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_version_extend_private_owned(
	struct gpu_m4_drm_provider_version *source,
	const struct gpu_m4_drm_uat_private *private,
	struct gpu_m4_drm_provider_private_lease *const *leases, u32 count, bool initialize)
{
	struct gpu_m4_drm_provider_version *v;
	struct gpu_m4_drm_provider *p;
	int ret;

	if (!source || (count && !private) || count > GPU_M4_DRM_MAX_BINDS ||
	    source->private_count > GPU_M4_DRM_MAX_BINDS - count)
		return ERR_PTR(-EINVAL);
	p = source->vm->provider;
	lockdep_assert_held(p->lock);
	if (p->error)
		return ERR_PTR(p->error);
	/* These are externally retained allocator leases, never raw user PAs or
	 * implicit upper-address transforms. The builder proves backing and aliases.
	 */
	for (u32 i = 0; i < count; i++)
		if (!private[i].identity || !private[i].cookie || !private[i].get ||
		    !private[i].put || !private[i].page ||
		    !gpu_m4_drm_aligned(private[i].va) ||
		    !gpu_m4_drm_aligned(private[i].size) ||
		    !gpu_m4_drm_range(private[i].va, private[i].size,
			GPU_M4_DRM_UAT_PRIVATE_START, GPU_M4_DRM_UAT_PRIVATE_END))
			return ERR_PTR(-EINVAL);
	/* An immutable source GRAPH is adopted by its constructed identity. */
	ret = source->state == GPU_M4_PROVIDER_VERSION_GRAPH ?
		gpu_m4_drm_uat_check(source->uat) :
		(source->state == GPU_M4_PROVIDER_VERSION_SNAPSHOT ? 0 : -EINVAL);
	if (!ret && !initialize)
		ret = gpu_m4_drm_provider_graph_base_locked(source);
	if (ret)
		return ERR_PTR(ret);
	v = gpu_m4_drm_provider_version_alloc(source->vm, source->ledger.count,
		source->object_count, source->private_count + count);
	if (IS_ERR(v))
		return v;
	ret = gpu_m4_drm_provider_version_copy_private(v, source);
	if (count)
		memcpy(v->private + source->private_count, private, array_size(count, sizeof(*private)));
	for (u32 i = 0; !ret && leases && i < count; i++) {
		ret = gpu_m4_drm_provider_private_check_locked(leases[i], source->vm,
			private[i].va, private[i].size);
		if (!ret)
			ret = gpu_m4_drm_provider_private_get_locked(leases[i]);
		if (!ret)
			v->private_leases[v->private_lease_count++] = leases[i];
	}
	if (!ret)
		ret = gpu_m4_drm_provider_version_build(v, initialize ? NULL : source);
	if (!ret && !initialize && !gpu_m4_drm_provider_version_same_user_locked(v, source))
		ret = -EPROTO;
	if (ret) {
		gpu_m4_drm_provider_version_put_locked(v);
		return ERR_PTR(ret);
	}
	return v;
}

struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_version_extend_private_locked(
	struct gpu_m4_drm_provider_version *source,
	const struct gpu_m4_drm_uat_private *private, u32 count)
{
	return gpu_m4_drm_provider_version_extend_private_owned(source, private, NULL, count, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_extend_private_locked);

int gpu_m4_drm_provider_vm_initialize_private_locked(
	struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_uat_private *private,
	struct gpu_m4_drm_provider_private_lease *const *leases, u32 count)
{
	struct gpu_m4_drm_provider_version *old, *v;
	struct gpu_m4_drm_provider *p;

	if (!vm || !private || !leases || !count)
		return -EINVAL;
	p = vm->provider;
	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm))
		return p->error ?: -ENOENT;
	old = vm->active;
	if (!old || vm->generation != 1 || old->generation != 1 ||
	    old->ledger.count || old->private_count != 1 || old->native_private_count ||
	    old->private_lease_count || refcount_read(&old->refs) != 1 ||
	    old->protected || old->working || old->binding_asid)
		return -EBUSY;
	/* No accepted capture exists yet: create this initial baseline's own user
	 * lineage, rather than retaining an obsolete empty snapshot indefinitely.
	 */
	v = gpu_m4_drm_provider_version_extend_private_owned(old, private, leases, count, true);
	if (IS_ERR(v))
		return PTR_ERR(v);
	v->base_private_count = v->private_count; /* the VM's common private baseline */
	gpu_m4_drm_provider_vm_withdraw_locked(vm);
	vm->active = v;
	gpu_m4_drm_provider_version_put_locked(old);
	/* S3c: the globals are DRIVER extents of the authority tree (strict). */
	if (vm->authority)
		return gpu_m4_drm_provider_tree_authority_private_locked(vm,
			v->private + v->private_count - count, count);
	/* The UMA/parameter globals join the shadow tree as DRIVER extents. */
	gpu_m4_drm_provider_tree_private_locked(vm, v->private + v->private_count - count, count);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_initialize_private_locked);

struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_version_extend_private_gem_locked(
	struct gpu_m4_drm_provider_version *source,
	const struct gpu_m4_drm_provider_private_gem *private, u32 count)
{
	struct gpu_m4_drm_provider_version *v;
	struct gpu_m4_drm_provider *p;
	int ret;

	if (!source || (count && !private) || count > GPU_M4_DRM_MAX_BINDS ||
	    source->private_count > GPU_M4_DRM_MAX_BINDS - count)
		return ERR_PTR(-EINVAL);
	p = source->vm->provider;
	lockdep_assert_held(p->lock);
	if (p->error || !refcount_read(&source->refs) || (count && source->vm->closed))
		return ERR_PTR(p->error ? p->error : -ENOENT);
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_provider_private_gem *m = &private[i];

		if (!gpu_m4_drm_provider_object_origin(m->object, source->vm))
			return ERR_PTR(-EPERM);
		ret = gpu_m4_drm_provider_private_check_locked(m->lease, source->vm, m->va, m->size);
		if (ret)
			return ERR_PTR(ret);
		if (!gpu_m4_drm_aligned(m->offset) ||
		    !gpu_m4_drm_aligned(m->size) || !gpu_m4_drm_vm_leaf_attrs(m->access) ||
		    m->offset >= m->object->backing.size ||
		    m->size > m->object->backing.size - m->offset)
			return ERR_PTR(-EINVAL);
	}
	/* An immutable source GRAPH is adopted by its constructed identity; a
	 * SNAPSHOT source first gets the one complete graph its extensions share.
	 */
	ret = source->state == GPU_M4_PROVIDER_VERSION_GRAPH ?
		gpu_m4_drm_uat_check(source->uat) :
		(source->state == GPU_M4_PROVIDER_VERSION_SNAPSHOT ? 0 : -EINVAL);
	if (!ret)
		ret = gpu_m4_drm_provider_graph_base_locked(source);
	if (ret)
		return ERR_PTR(ret);
	v = gpu_m4_drm_provider_version_alloc(source->vm, source->ledger.count,
		source->object_count + count, source->private_count + count);
	if (IS_ERR(v))
		return v;
	ret = gpu_m4_drm_provider_version_copy_private(v, source);
	for (u32 i = 0; !ret && i < count; i++) {
		const struct gpu_m4_drm_provider_private_gem *m = &private[i];
		struct gpu_m4_drm_provider_object *o = m->object;
		bool held = false;

		if (!gpu_m4_drm_provider_version_object(v, o->backing.identity)) {
			ret = gpu_m4_drm_provider_object_get_locked(o);
			if (ret)
				break;
			v->objects[v->object_count++] = o;
		}
		for (u32 j = 0; j < v->private_lease_count; j++)
			if (v->private_leases[j] == m->lease)
				held = true;
		if (!held) {
			ret = gpu_m4_drm_provider_private_get_locked(m->lease);
			if (ret)
				break;
			v->private_leases[v->private_lease_count++] = m->lease;
		}
		v->private[source->private_count + i] = (struct gpu_m4_drm_uat_private) {
			.identity = o->backing.identity, .cookie = o,
			.get = gpu_m4_drm_provider_native_get, .put = gpu_m4_drm_provider_native_put,
			.page = gpu_m4_drm_provider_native_page,
			.va = m->va, .offset = m->offset, .size = m->size,
			.access = m->access, .cpu_policy = o->backing.cpu_policy,
		};
		v->native_private[v->native_private_count++] = (struct gpu_m4_drm_cache_access) {
			.pin = o->pin, .offset = m->offset, .size = m->size, .access = m->access,
		};
	}
	if (!ret)
		ret = gpu_m4_drm_provider_version_build(v, source);
	if (!ret && !gpu_m4_drm_provider_version_same_user_locked(v, source))
		ret = -EPROTO;
	if (ret) {
		gpu_m4_drm_provider_version_put_locked(v);
		return ERR_PTR(ret);
	}
	return v;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_extend_private_gem_locked);

/* ---- Persistent private native baseline (mapping-only membership) ---- */

static void gpu_m4_drm_provider_baseline_put_locked(struct gpu_m4_drm_provider_baseline *b)
{
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_provider *p;

	if (!b)
		return;
	p = b->vm->provider;
	lockdep_assert_held(p->lock);
	if (!refcount_dec_and_test(&b->refs))
		return;
	/* Membership only: drop exactly its own object/claim references through
	 * the ordinary deferred reapers; never a physical release here.
	 */
	for (u32 i = 0; i < b->object_count; i++)
		gpu_m4_drm_provider_object_put_locked(b->objects[i]);
	for (u32 i = 0; i < b->lease_count; i++)
		gpu_m4_drm_provider_private_put_locked(b->leases[i]);
	p->bytes -= b->charge;
	vm = b->vm;
	kvfree(b->member);
	kvfree(b->objects);
	kvfree(b->leases);
	kfree(b);
	/* Last: the VM reference, after charge and member references. */
	gpu_m4_drm_provider_vm_put_locked(vm);
}

static int gpu_m4_drm_provider_baseline_member_compare(const void *first, const void *second)
{
	const struct gpu_m4_drm_provider_baseline_member *a = first, *b = second;

	return a->va < b->va ? -1 : a->va > b->va;
}

static bool gpu_m4_drm_provider_baseline_member_same(
	const struct gpu_m4_drm_provider_baseline_member *a,
	const struct gpu_m4_drm_provider_private_gem *m)
{
	return a->object == m->object && a->lease == m->lease && a->va == m->va &&
		a->offset == m->offset && a->size == m->size && a->access == m->access;
}

/* Exact member lookup by VA (members never overlap), or NULL. */
static const struct gpu_m4_drm_provider_baseline_member *gpu_m4_drm_provider_baseline_find(
	const struct gpu_m4_drm_provider_baseline *b, const struct gpu_m4_drm_provider_private_gem *m)
{
	u32 first = 0, end = b ? b->count : 0;

	while (first < end) {
		u32 middle = first + (end - first) / 2;

		if (b->member[middle].va < m->va)
			first = middle + 1;
		else
			end = middle;
	}
	if (b && first < b->count && gpu_m4_drm_provider_baseline_member_same(&b->member[first], m))
		return &b->member[first];
	return NULL;
}

/*
 * Distinct member objects in identity order without sorting the unchanged
 * ones: the old baseline's (identity-ordered) objects still named by a member,
 * then the newly named ones sorted on their own and merged in once. Members
 * carry the current stamp; kept old objects are restamped so only new ones
 * remain. No references are taken here.
 */
static int gpu_m4_drm_provider_baseline_objects(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_baseline *b, const struct gpu_m4_drm_provider_baseline *old,
	u64 stamp)
{
	struct gpu_m4_drm_provider_object **merged;
	u32 kept = 0, fresh = 0, i, j, k;

	for (i = 0; old && i < old->object_count; i++)
		if (old->objects[i]->baseline_stamp == stamp)
			b->objects[kept++] = old->objects[i];
	p->baseline_stamp++;
	for (i = 0; i < kept; i++)
		b->objects[i]->baseline_stamp = p->baseline_stamp;
	for (i = 0; i < b->count; i++) {
		struct gpu_m4_drm_provider_object *o = b->member[i].object;

		if (o->baseline_stamp == stamp) {
			o->baseline_stamp = p->baseline_stamp;
			b->objects[kept + fresh++] = o;
		}
	}
	b->object_count = kept + fresh;
	if (!fresh)
		return 0;
	sort(b->objects + kept, fresh, sizeof(*b->objects), gpu_m4_drm_provider_object_compare,
	     NULL);
	if (!kept)
		return 0;
	merged = kvcalloc(kept + fresh, sizeof(*merged), GFP_KERNEL);
	if (!merged)
		return -ENOMEM;
	for (i = 0, j = kept, k = 0; i < kept || j < kept + fresh; k++)
		merged[k] = j == kept + fresh || (i < kept &&
			gpu_m4_drm_provider_object_compare(&b->objects[i], &b->objects[j]) < 0) ?
			b->objects[i++] : b->objects[j++];
	memcpy(b->objects, merged, array_size(kept + fresh, sizeof(*merged)));
	kvfree(merged);
	return 0;
}

/*
 * Build the next immutable baseline from `old` (may be NULL) minus `remove`
 * plus `add` (both exact typed descriptors). The new baseline takes its own
 * references on every distinct object (identity order) and claim (first-
 * appearance order). Charged; no installation. old is VA-sorted: removal
 * keeps that order, the additions alone are sorted and merged in once.
 */
static struct gpu_m4_drm_provider_baseline *gpu_m4_drm_provider_baseline_next(
	struct gpu_m4_drm_provider_vm *vm, const struct gpu_m4_drm_provider_baseline *old,
	const struct gpu_m4_drm_provider_private_gem *add, u32 add_count,
	const struct gpu_m4_drm_provider_private_gem *remove, u32 remove_count)
{
	struct gpu_m4_drm_provider *p = vm->provider;
	struct gpu_m4_drm_provider_baseline *b;
	u32 old_count = old ? old->count : 0, n = 0;
	u64 total = (u64)old_count + add_count, charge;
	int ret;

	if (total > GPU_M4_DRM_MAX_BINDS)
		return ERR_PTR(-E2BIG);
	charge = sizeof(*b) + (total ? total : 1) * (sizeof(*b->member) + sizeof(*b->objects) +
						    sizeof(*b->leases));
	ret = gpu_m4_drm_provider_charge(p, charge);
	if (ret)
		return ERR_PTR(ret);
	b = kzalloc_obj(*b);
	if (b) {
		b->member = kvcalloc(total ? total : 1, sizeof(*b->member), GFP_KERNEL);
		b->objects = kvcalloc(total ? total : 1, sizeof(*b->objects), GFP_KERNEL);
		b->leases = kvcalloc(total ? total : 1, sizeof(*b->leases), GFP_KERNEL);
	}
	if (!b || !b->member || !b->objects || !b->leases) {
		if (b) {
			kvfree(b->member);
			kvfree(b->objects);
			kvfree(b->leases);
		}
		kfree(b);
		p->bytes -= charge;
		return ERR_PTR(-ENOMEM);
	}
	refcount_set(&b->refs, 1);
	/* Every baseline, empty ones included, holds its VM for its lifetime. */
	gpu_m4_drm_provider_vm_get_locked(vm);
	b->vm = vm;
	b->charge = charge;
	for (u32 i = 0; i < old_count; i++) {
		bool removed = false;

		for (u32 j = 0; j < remove_count && !removed; j++)
			removed = gpu_m4_drm_provider_baseline_member_same(&old->member[i], &remove[j]);
		if (!removed)
			b->member[n++] = old->member[i];
	}
	/* Additions (admission revision assigned at install) are sorted among
	 * themselves at the tail, then merged backward with the kept members.
	 */
	for (u32 i = 0; i < add_count; i++)
		b->member[n + i] = (struct gpu_m4_drm_provider_baseline_member) {
			.object = add[i].object, .lease = add[i].lease, .va = add[i].va,
			.offset = add[i].offset, .size = add[i].size, .access = add[i].access,
		};
	if (add_count > 1)
		sort(b->member + n, add_count, sizeof(*b->member),
		     gpu_m4_drm_provider_baseline_member_compare, NULL);
	if (n && add_count) {
		struct gpu_m4_drm_provider_baseline_member *added;

		added = kvcalloc(add_count, sizeof(*added), GFP_KERNEL);
		if (!added) {
			gpu_m4_drm_provider_baseline_put_locked(b);
			return ERR_PTR(-ENOMEM);
		}
		memcpy(added, b->member + n, array_size(add_count, sizeof(*added)));
		for (u32 i = n, j = add_count, k = n + add_count; j; ) {
			if (i && gpu_m4_drm_provider_baseline_member_compare(&b->member[i - 1],
									      &added[j - 1]) > 0)
				b->member[--k] = b->member[--i];
			else
				b->member[--k] = added[--j];
		}
		kvfree(added);
	}
	n += add_count;
	b->count = n;
	/* One fresh stamp per build marks every object/claim already listed:
	 * the same distinct sets as comparing against the list.
	 */
	p->baseline_stamp++;
	for (u32 i = 0; i < n; i++) {
		struct gpu_m4_drm_provider_baseline_member *m = &b->member[i];

		b->pages += m->size / GPU_M4_DRM_PAGE_SIZE;
		/* Callers proved liveness/origin of every object and claim. */
		m->object->baseline_stamp = p->baseline_stamp;
		if (m->lease->baseline_stamp != p->baseline_stamp) {
			m->lease->baseline_stamp = p->baseline_stamp;
			gpu_m4_drm_provider_private_get_locked(m->lease);
			b->leases[b->lease_count++] = m->lease;
		}
	}
	if (gpu_m4_drm_provider_baseline_objects(p, b, old, p->baseline_stamp)) {
		b->object_count = 0; /* no object reference taken yet */
		gpu_m4_drm_provider_baseline_put_locked(b);
		return ERR_PTR(-ENOMEM);
	}
	for (u32 i = 0; i < b->object_count; i++)
		gpu_m4_drm_provider_object_get_locked(b->objects[i]);
	return b;
}

/* Install b as the VM's next baseline (consumes the caller's reference). */
static u64 gpu_m4_drm_provider_baseline_install_locked(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_baseline *b)
{
	struct gpu_m4_drm_provider_baseline *old = vm->baseline;

	b->revision = ++vm->baseline_revision;
	for (u32 i = 0; i < b->count; i++)
		if (!b->member[i].since)
			b->member[i].since = b->revision;
	vm->baseline = b;
	gpu_m4_drm_provider_baseline_put_locked(old);
	return b->revision;
}

static bool gpu_m4_drm_provider_range_overlap(u64 a, u64 a_size, u64 b, u64 b_size)
{
	return a < b + b_size && b < a + a_size;
}

/* Does [va, va + size) overlap a member? Members are VA-sorted and pairwise
 * disjoint, so their ends ascend too: only the last member starting below
 * the range's end can reach into it.
 */
static bool gpu_m4_drm_provider_baseline_overlap(const struct gpu_m4_drm_provider_baseline *b,
	u64 va, u64 size)
{
	u32 first = 0, end = b ? b->count : 0;

	while (first < end) {
		u32 middle = first + (end - first) / 2;

		if (b->member[middle].va < va + size)
			first = middle + 1;
		else
			end = middle;
	}
	return first && gpu_m4_drm_provider_range_overlap(va, size, b->member[first - 1].va,
							  b->member[first - 1].size);
}

/*
 * Validate one typed mapped descriptor of a completed unit exactly as the
 * per-child private append does (origin/pin, claim and guards, bounds,
 * permission, every PA through the pin's admitted oracle) and its collisions
 * with the common private baseline and every other (existing or new) member.
 */
static int gpu_m4_drm_provider_baseline_admit_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_baseline *old,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 index)
{
	const struct gpu_m4_drm_provider_private_gem *m = &maps[index];
	struct gpu_m4_drm_provider_version *active = vm->active;
	u64 pa;
	int ret;

	if (!gpu_m4_drm_provider_object_origin(m->object, vm))
		return -EPERM;
	ret = gpu_m4_drm_provider_private_check_locked(m->lease, vm, m->va, m->size);
	if (ret)
		return ret;
	if (!m->size || !gpu_m4_drm_aligned(m->offset) || !gpu_m4_drm_aligned(m->size) ||
	    !gpu_m4_drm_vm_leaf_attrs(m->access) || m->offset >= m->object->backing.size ||
	    m->size > m->object->backing.size - m->offset)
		return -EINVAL;
	for (u64 offset = 0; offset < m->size; offset += GPU_M4_DRM_PAGE_SIZE) {
		ret = gpu_m4_drm_provider_object_page_locked(m->object, m->offset + offset, &pa);
		if (ret)
			return ret;
	}
	for (u32 i = 0; active && i < active->base_private_count; i++)
		if (gpu_m4_drm_provider_range_overlap(m->va, m->size, active->private[i].va,
						      active->private[i].size))
			return -EADDRINUSE;
	if (gpu_m4_drm_provider_baseline_overlap(old, m->va, m->size))
		return -EADDRINUSE;
	for (u32 i = 0; i < index; i++)
		if (gpu_m4_drm_provider_range_overlap(m->va, m->size, maps[i].va, maps[i].size))
			return -EADDRINUSE;
	return 0;
}

int gpu_m4_drm_provider_vm_baseline_add_locked(struct gpu_m4_drm_provider_vm *vm,
	u64 expected_revision, const struct gpu_m4_drm_provider_private_gem *maps, u32 count,
	u64 *revision)
{
	struct gpu_m4_drm_provider_baseline *b;
	struct gpu_m4_drm_provider *p;
	u64 pages = 0;
	int ret;

	if (!vm || !maps || !count || count > GPU_M4_DRM_MAX_BINDS || !revision)
		return -EINVAL;
	p = vm->provider;
	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm))
		return p->error ?: -ENOENT;
	/* Exact private-revision recheck: a racing commit forces the caller to
	 * re-resolve against the newer baseline. Failure changes nothing.
	 */
	if (expected_revision != vm->baseline_revision || !vm->active)
		return -EAGAIN;
	for (u32 i = 0; i < count; i++) {
		ret = gpu_m4_drm_provider_baseline_admit_locked(vm, vm->baseline, maps, i);
		if (ret)
			return ret;
		pages += maps[i].size / GPU_M4_DRM_PAGE_SIZE;
	}
	if ((vm->baseline ? vm->baseline->pages : 0) + pages + vm->active->ledger.count >
	    (u64)p->config.max_leaves)
		return -ENOSPC;
	b = gpu_m4_drm_provider_baseline_next(vm, vm->baseline, maps, count, NULL, 0);
	if (IS_ERR(b))
		return PTR_ERR(b);
	*revision = gpu_m4_drm_provider_baseline_install_locked(vm, b);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_baseline_add_locked);

int gpu_m4_drm_provider_vm_baseline_remove_locked(struct gpu_m4_drm_provider_vm *vm,
	u64 expected_revision, const struct gpu_m4_drm_provider_private_gem *maps, u32 count,
	u64 *revision)
{
	struct gpu_m4_drm_provider_baseline *b;
	struct gpu_m4_drm_provider *p;

	if (!vm || !maps || !count || count > GPU_M4_DRM_MAX_BINDS || !revision)
		return -EINVAL;
	p = vm->provider;
	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm))
		return p->error ?: -ENOENT;
	if (expected_revision != vm->baseline_revision)
		return -EAGAIN;
	for (u32 i = 0; i < count; i++) {
		if (!gpu_m4_drm_provider_baseline_find(vm->baseline, &maps[i]))
			return -ENOENT;
		for (u32 j = 0; j < i; j++)
			if (maps[j].va == maps[i].va)
				return -EINVAL;
	}
	/* Future membership only: holders of the old baseline (composed job
	 * versions, captures) keep exactly their mappings and backing. This is
	 * the first step of every eviction: a smaller successor replacing the
	 * old record, so charge pressure must not refuse the reclaim itself.
	 */
	p->replacing = vm->baseline->charge;
	b = gpu_m4_drm_provider_baseline_next(vm, vm->baseline, NULL, 0, maps, count);
	p->replacing = 0;
	if (IS_ERR(b))
		return PTR_ERR(b);
	*revision = gpu_m4_drm_provider_baseline_install_locked(vm, b);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_baseline_remove_locked);

u64 gpu_m4_drm_provider_vm_baseline_revision_locked(struct gpu_m4_drm_provider_vm *vm)
{
	lockdep_assert_held(vm->provider->lock);
	return vm->baseline_revision;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_baseline_revision_locked);

struct gpu_m4_drm_provider_baseline *gpu_m4_drm_provider_vm_baseline_get_locked(
	struct gpu_m4_drm_provider_vm *vm)
{
	if (!vm)
		return NULL;
	lockdep_assert_held(vm->provider->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm) || !vm->baseline)
		return NULL;
	refcount_inc(&vm->baseline->refs);
	return vm->baseline;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_baseline_get_locked);

void gpu_m4_drm_provider_baseline_release_locked(struct gpu_m4_drm_provider_baseline *b)
{
	gpu_m4_drm_provider_baseline_put_locked(b);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_baseline_release_locked);

u64 gpu_m4_drm_provider_baseline_revision(const struct gpu_m4_drm_provider_baseline *b)
{
	return b ? b->revision : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_baseline_revision);

bool gpu_m4_drm_provider_baseline_contains_locked(const struct gpu_m4_drm_provider_baseline *b,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count)
{
	if (!b || (count && !maps))
		return false;
	lockdep_assert_held(b->vm->provider->lock);
	for (u32 i = 0; i < count; i++)
		if (!gpu_m4_drm_provider_baseline_find(b, &maps[i]))
			return false;
	return true;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_baseline_contains_locked);

/* REMOVE-DEBUG: holder census (see the header). Scratch fields only. */
u64 gpu_m4_drm_provider_census_begin_locked(struct gpu_m4_drm_provider *p)
{
	lockdep_assert_held(p->lock);
	return ++p->census_stamp;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_census_begin_locked);

static void gpu_m4_drm_provider_census_mark(struct gpu_m4_drm_provider_object *o, u64 stamp,
	u32 holder)
{
	if (o->census == stamp)
		return;
	o->census = stamp;
	o->census_holder = holder;
}

void gpu_m4_drm_provider_object_census_locked(struct gpu_m4_drm_provider_object *o, u64 stamp)
{
	lockdep_assert_held(o->vm->provider->lock);
	gpu_m4_drm_provider_census_mark(o, stamp, GPU_M4_DRM_PROVIDER_HOLDER_CALLER);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_object_census_locked);

static void gpu_m4_drm_provider_census_baseline(const struct gpu_m4_drm_provider_baseline *b,
	u64 stamp, u32 holder)
{
	for (u32 i = 0; b && i < b->object_count; i++)
		gpu_m4_drm_provider_census_mark(b->objects[i], stamp, holder);
}

void gpu_m4_drm_provider_holders_locked(struct gpu_m4_drm_provider *p, u64 stamp,
	struct gpu_m4_drm_provider_holders *h)
{
	struct gpu_m4_drm_provider_object *o;
	struct gpu_m4_drm_provider_version *v;
	struct gpu_m4_drm_provider_vm *vm;

	lockdep_assert_held(p->lock);
	*h = (struct gpu_m4_drm_provider_holders) {
		.images_released = p->images_released, .image_objects = p->image_objects,
		.image_failures = p->image_failures,
	};
	list_for_each_entry(vm, &p->vms, link) {
		gpu_m4_drm_provider_census_baseline(vm->baseline, stamp,
			GPU_M4_DRM_PROVIDER_HOLDER_BASELINE);
		/* An object both a live composition and a finished one hold
		 * stays held by the live one: classify that first.
		 */
		list_for_each_entry(v, &vm->versions, link) {
			h->composed += !!v->baseline;
			h->finished += v->baseline && v->finished;
			h->retired += v->state == GPU_M4_PROVIDER_VERSION_RETIRED;
			if (v->baseline != vm->baseline && !v->finished)
				gpu_m4_drm_provider_census_baseline(v->baseline, stamp,
					GPU_M4_DRM_PROVIDER_HOLDER_COMPOSED);
		}
		list_for_each_entry(v, &vm->versions, link)
			if (v->baseline != vm->baseline && v->finished)
				gpu_m4_drm_provider_census_baseline(v->baseline, stamp,
					GPU_M4_DRM_PROVIDER_HOLDER_FINISHED);
		list_for_each_entry(v, &vm->versions, link)
			for (u32 i = 0; i < v->object_count; i++)
				gpu_m4_drm_provider_census_mark(v->objects[i], stamp,
					GPU_M4_DRM_PROVIDER_HOLDER_VERSION);
		list_for_each_entry(o, &vm->objects, link) {
			gpu_m4_drm_provider_census_mark(o, stamp, GPU_M4_DRM_PROVIDER_HOLDER_OTHER);
			h->bytes[o->census_holder] += gpu_m4_drm_bo_private_bytes(
				(struct drm_gem_object *)o->backing.identity);
			h->objects[o->census_holder]++;
		}
	}
	list_for_each_entry(o, &p->garbage_objects, garbage) {
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_GARBAGE] += gpu_m4_drm_bo_private_bytes(
			(struct drm_gem_object *)o->backing.identity);
		h->objects[GPU_M4_DRM_PROVIDER_HOLDER_GARBAGE]++;
	}
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_holders_locked);

/*
 * The child's actual version: source's exact held user map (its immutable
 * ledger and index borrowed under a source reference, not copied) and common
 * private baseline, plus EVERY member of the held native baseline as
 * mapping-only private mappings. No native_private[] access: the snapshot is
 * a zero-extra clone of the source capture, so unselected members never enter
 * cache/resv sets; the caller joins exactly its selected accesses. Returned
 * in SNAPSHOT state; its graph is shared/derived later (ensure_graph).
 * -ESTALE when the source cannot be composed (caller keeps the per-child
 * append path).
 */
struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_version_compose_locked(
	struct gpu_m4_drm_provider_version *source, struct gpu_m4_drm_provider_baseline *b)
{
	struct gpu_m4_drm_provider_version *v;
	struct gpu_m4_drm_provider *p;
	u32 base;
	int ret;

	if (IS_ERR_OR_NULL(source) || IS_ERR_OR_NULL(b) || b->vm != source->vm)
		return ERR_PTR(-EINVAL);
	p = source->vm->provider;
	lockdep_assert_held(p->lock);
	if (p->error || !refcount_read(&source->refs) || source->vm->closed)
		return ERR_PTR(p->error ?: -ENOENT);
	base = source->base_private_count;
	/* Only a plain common-baseline source: never an already composed version
	 * (its member claims/objects would survive removal as ghosts) nor any
	 * extension (native accesses or external private mappings would be
	 * dropped). Those keep the per-child append path. Rejected before any
	 * allocation or reference. The common claim inventory plus the members'
	 * claims must fit the version's claim array as allocated and charged.
	 */
	if (source->state < GPU_M4_PROVIDER_VERSION_SNAPSHOT ||
	    source->state > GPU_M4_PROVIDER_VERSION_GRAPH || !source->objects_indexed ||
	    source->native_private_count ||
	    source->baseline || !base || source->private_count != base ||
	    b->count > GPU_M4_DRM_MAX_BINDS - base || b->lease_count > b->count ||
	    source->private_lease_count > base)
		return ERR_PTR(-ESTALE);
	/* The source's committed user ledger and its retained index are
	 * immutable: v borrows them under a version reference (charged only
	 * once, by the source) instead of copying O(user leaves) per child.
	 */
	ret = gpu_m4_drm_provider_version_get_locked(source);
	if (ret)
		return ERR_PTR(ret);
	v = gpu_m4_drm_provider_version_alloc_storage(source->vm, 0,
		source->object_count + b->object_count, base + b->count, source->ledger.leaf);
	if (IS_ERR(v)) {
		gpu_m4_drm_provider_version_put_locked(source);
		return v;
	}
	v->ledger_source = source;
	v->ledger.count = source->ledger.count;
	v->ledger.capacity = source->ledger.capacity;
	if (gpu_m4_drm_provider_version_indexed(source))
		v->index = source->index;
	ret = 0;
	memcpy(v->private, source->private, array_size(base, sizeof(*v->private)));
	/* Both inventories are identity-ordered (the indexed SNAPSHOT source's
	 * and the baseline's distinct objects, one per identity of this VM): one
	 * merge, the source's object standing for an identity both name, yields
	 * this version's sorted inventory without a search or sort. A duplicate
	 * identity would still fail this version's own index check.
	 */
	for (u32 i = 0, j = 0; !ret && (i < source->object_count || j < b->object_count);) {
		struct gpu_m4_drm_provider_object *o;
		int order = i == source->object_count ? 1 : j == b->object_count ? -1 :
			gpu_m4_drm_provider_object_compare(&source->objects[i], &b->objects[j]);

		if (!order)
			j++;
		o = order <= 0 ? source->objects[i++] : b->objects[j++];
		ret = gpu_m4_drm_provider_object_get_locked(o);
		if (!ret)
			v->objects[v->object_count++] = o;
	}
	v->objects_sorted = v->object_count;
	for (u32 i = 0; !ret && i < source->private_lease_count; i++) {
		ret = gpu_m4_drm_provider_private_get_locked(source->private_leases[i]);
		if (!ret)
			v->private_leases[v->private_lease_count++] = source->private_leases[i];
	}
	/* Claim references for the composed members (guards still enforced);
	 * the admission bound above keeps them within the charged array. The
	 * baseline's claims are pairwise distinct by construction: only the
	 * copied source prefix can already hold one.
	 */
	for (u32 i = 0; !ret && i < b->lease_count; i++) {
		bool held = false;

		for (u32 j = 0; j < source->private_lease_count; j++)
			held |= v->private_leases[j] == b->leases[i];
		if (held)
			continue;
		ret = gpu_m4_drm_provider_private_get_locked(b->leases[i]);
		if (!ret)
			v->private_leases[v->private_lease_count++] = b->leases[i];
	}
	for (u32 i = 0; !ret && i < b->count; i++) {
		const struct gpu_m4_drm_provider_baseline_member *m = &b->member[i];

		v->private[base + i] = (struct gpu_m4_drm_uat_private) {
			.identity = m->object->backing.identity, .cookie = m->object,
			.get = gpu_m4_drm_provider_native_get, .put = gpu_m4_drm_provider_native_put,
			.page = gpu_m4_drm_provider_native_page,
			.va = m->va, .offset = m->offset, .size = m->size,
			.access = m->access, .cpu_policy = m->object->backing.cpu_policy,
		};
	}
	if (!ret) {
		refcount_inc(&b->refs);
		v->baseline = b;
		v->base_private_count = base;
		v->baseline_revision = source->baseline_revision;
		v->generation = source->generation;
		ret = gpu_m4_drm_provider_version_snapshot(v, source);
	}
	if (!ret && !gpu_m4_drm_provider_version_same_user_locked(v, source))
		ret = -EPROTO;
	if (ret) {
		gpu_m4_drm_provider_version_put_locked(v);
		return ERR_PTR(ret);
	}
	gpu_m4_drm_provider_tree_composed_locked(v);
	return v;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_compose_locked);

/*
 * Selected-access proof for a composed child: every selected typed native
 * descriptor is an exact member of the candidate's held baseline, and the
 * child's actual captured (joined) cache/resv rows cover its range and
 * permission with the same pin. Together with native_covered_locked() (the
 * unchanged user/timestamp prefix) this is the complete coverage proof.
 */
bool gpu_m4_drm_provider_version_selected_covered_locked(
	struct gpu_m4_drm_provider_version *candidate,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count,
	const struct gpu_m4_drm_vm_owner *captured)
{
	const struct gpu_m4_drm_cache_access *have;
	const struct gpu_m4_drm_resv_access *have_resv;
	u32 h, hr;

	if (!candidate || !captured || (count && !maps) || !candidate->baseline)
		return false;
	lockdep_assert_held(candidate->vm->provider->lock);
	have = gpu_m4_drm_vm_owner_cache_access(captured, &h);
	have_resv = gpu_m4_drm_vm_owner_resv_access(captured, &hr);
	if ((h && !have) || (hr && !have_resv) ||
	    gpu_m4_drm_vm_owner_vm(captured) != candidate->vm->frontend)
		return false;
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_provider_private_gem *m = &maps[i];
		struct drm_gem_object *object;
		bool cache = false, reservation = false;

		if (!gpu_m4_drm_provider_baseline_find(candidate->baseline, m))
			return false;
		object = (struct drm_gem_object *)m->object->backing.identity;
		for (u32 j = 0; j < h && !cache; j++)
			cache = have[j].pin == m->object->pin && have[j].offset <= m->offset &&
				m->offset - have[j].offset < have[j].size &&
				m->size <= have[j].size - (m->offset - have[j].offset) &&
				!(m->access & ~have[j].access);
		for (u32 j = 0; j < hr && !reservation; j++)
			reservation = have_resv[j].object == object &&
				!(m->access & ~have_resv[j].access);
		if (!cache || !reservation)
			return false;
	}
	return true;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_selected_covered_locked);

/*
 * REMOVE-DEBUG until S3c: the shadow persistent per-VM tree. It mirrors every
 * committed change of the VM's composed image, after the version path (and
 * the native baseline) committed it, and is never installed: its TLBI
 * callbacks issue nothing. User leaves follow vm->active (VM_BIND, GEM
 * close); DRIVER extents follow the common private baseline (the context at
 * VM create, the UMA/parameter globals at initialize_private) and the native
 * baseline (tree_native_add/remove, called right after each committed
 * baseline add/remove). The version path and the baselines stay the
 * authority. Any tree failure desyncs (empties) the tree and changes neither
 * the caller's result nor anything the GPU can see. Mirroring runs under the
 * provider mutex (the tree mutex nests inside it), and the tree's references
 * are ordinary provider object references (native objects included); the
 * context and the globals are DRIVER extents without an object.
 */
static int gpu_m4_drm_provider_tree_get(void *cookie __always_unused, void *object)
{
	return gpu_m4_drm_provider_object_get_locked(object);
}

static void gpu_m4_drm_provider_tree_put(void *cookie __always_unused, void *object)
{
	gpu_m4_drm_provider_object_put_locked(object);
}

/* A live SHADOW tree. An authority tree is never mirrored, compared or
 * desynced by these hooks: its own section below edits it strictly.
 */
static bool gpu_m4_drm_provider_tree_live(struct gpu_m4_drm_provider_vm *vm)
{
	lockdep_assert_held(vm->provider->lock);
	return vm->tree && !vm->authority && !gpu_m4_vm_tree_desynced(vm->tree);
}

/* Expected complete image of an oracle point, one word per valid leaf. */
struct gpu_m4_drm_provider_image {
	struct gpu_m4_vm_tree_word *word;
	u32 count, capacity;
};

static int gpu_m4_drm_provider_image_add(struct gpu_m4_drm_provider_image *image, u64 va,
	u64 pa, u32 access, bool driver)
{
	if (image->count == image->capacity || !gpu_m4_drm_vm_leaf_attrs(access))
		return -EPROTO;
	image->word[image->count++] = (struct gpu_m4_vm_tree_word) {
		.va = va, .pte = gpu_m4_uat_fmt_leaf_pte(pa, access), .driver = driver,
	};
	return 0;
}

/*
 * REMOVE-DEBUG mode 2, the composed-version oracle: the tree's complete
 * image equals v's user leaves, v's private mappings private[0, privates)
 * (resolved through their own page callbacks) and the members of b (NULL:
 * none) resolved through their pins. A resolution failure is not a tree
 * fault: the point is only counted as skipped. A mismatch desyncs the tree.
 */
static void gpu_m4_drm_provider_tree_oracle_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_version *v, u32 privates,
	const struct gpu_m4_drm_provider_baseline *b)
{
	struct gpu_m4_drm_provider_image image = { 0 };
	u64 capacity = v->ledger.count + (b ? b->pages : 0), va = 0;
	int ret = 0;

	for (u32 i = 0; i < privates; i++)
		capacity += v->private[i].size / GPU_M4_DRM_PAGE_SIZE;
	if (capacity > GPU_M4_DRM_UAT_MAX_LEAVES) {
		gpu_m4_vm_tree_oracle_skipped();
		return;
	}
	image.capacity = capacity;
	image.word = kvcalloc(max(image.capacity, 1U), sizeof(*image.word), GFP_KERNEL);
	if (!image.word) {
		gpu_m4_vm_tree_oracle_skipped();
		return;
	}
	for (u32 i = 0; !ret && i < v->ledger.count; i++) {
		const struct gpu_m4_drm_vm_leaf *leaf = &v->ledger.leaf[i];

		ret = gpu_m4_drm_provider_image_add(&image, leaf->va, leaf->pa, leaf->access, false);
	}
	for (u32 i = 0; !ret && i < privates; i++) {
		const struct gpu_m4_drm_uat_private *m = &v->private[i];

		for (u64 offset = 0; !ret && offset < m->size; offset += GPU_M4_DRM_PAGE_SIZE) {
			u64 pa;

			ret = m->page(m->cookie, m->offset + offset, &pa);
			if (!ret)
				ret = gpu_m4_drm_provider_image_add(&image, m->va + offset, pa,
								    m->access, true);
		}
	}
	for (u32 i = 0; !ret && b && i < b->count; i++) {
		const struct gpu_m4_drm_provider_baseline_member *m = &b->member[i];

		for (u64 offset = 0; !ret && offset < m->size; offset += GPU_M4_DRM_PAGE_SIZE) {
			u64 pa;

			ret = gpu_m4_drm_provider_object_page_locked(m->object, m->offset + offset,
								      &pa);
			if (!ret)
				ret = gpu_m4_drm_provider_image_add(&image, m->va + offset, pa,
								    m->access, true);
		}
	}
	if (!ret) {
		ret = gpu_m4_vm_tree_compare_words(vm->tree, image.word, image.count, &va);
		if (ret)
			pr_warn("GPU shadow VM tree differs from the composed image: generation=%llu baseline=%llu va=%#llx error=%d\n",
				v->generation, b ? b->revision : 0, va, ret);
		if (ret && ret != -ESTALE)
			gpu_m4_vm_tree_desync(vm->tree);
	} else {
		gpu_m4_vm_tree_oracle_skipped();
	}
	kvfree(image.word);
}

/* The VM's current composed image: active's user leaves and common private
 * baseline (its whole private inventory) plus the current native baseline.
 */
static void gpu_m4_drm_provider_tree_current_locked(struct gpu_m4_drm_provider_vm *vm)
{
	const struct gpu_m4_drm_provider_version *a = vm->active;

	if (!a || a->baseline || a->private_count != a->base_private_count) {
		gpu_m4_vm_tree_oracle_skipped();
		return;
	}
	gpu_m4_drm_provider_tree_oracle_locked(vm, a, a->private_count, vm->baseline);
}

/* After a mirrored change: optionally compare, else desync on failure. */
static void gpu_m4_drm_provider_tree_settle_locked(struct gpu_m4_drm_provider_vm *vm, int ret)
{
	if (!ret && gpu_m4_vm_tree_mode() > 1 && gpu_m4_drm_provider_tree_live(vm))
		gpu_m4_drm_provider_tree_current_locked(vm);
	if (!ret)
		return;
	pr_warn("GPU shadow VM tree desynced: generation=%llu error=%d\n", vm->generation, ret);
	gpu_m4_vm_tree_desync(vm->tree);
}

/*
 * Mirror private mappings (context, globals) as DRIVER extents without an
 * object, each page resolved through the mapping's own page callback, as the
 * version builder resolves it.
 */
static void gpu_m4_drm_provider_tree_private_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_uat_private *private, u32 count)
{
	struct gpu_m4_vm_tree_op *ops;
	u64 **pa;
	int ret = 0;

	lockdep_assert_held(vm->provider->lock);
	if (!count || !gpu_m4_drm_provider_tree_live(vm))
		return;
	ops = kcalloc(count, sizeof(*ops), GFP_KERNEL);
	pa = kcalloc(count, sizeof(*pa), GFP_KERNEL);
	if (!ops || !pa)
		ret = -ENOMEM;
	for (u32 i = 0; !ret && i < count; i++) {
		const struct gpu_m4_drm_uat_private *m = &private[i];
		u64 pages = m->size / GPU_M4_DRM_PAGE_SIZE;

		pa[i] = pages && pages <= GPU_M4_DRM_UAT_MAX_LEAVES ?
			kvcalloc(pages, sizeof(**pa), GFP_KERNEL) : NULL;
		if (!pa[i]) {
			ret = pages ? -ENOMEM : -EINVAL;
			break;
		}
		for (u64 j = 0; !ret && j < pages; j++)
			ret = m->page(m->cookie, m->offset + j * GPU_M4_DRM_PAGE_SIZE, &pa[i][j]);
		ops[i] = (struct gpu_m4_vm_tree_op) {
			.va = m->va, .size = m->size, .object_size = m->size, .pa = pa[i],
			.access = m->access, .flags = GPU_M4_VM_TREE_DRIVER,
			.identity = m->identity, .cpu_policy = m->cpu_policy,
		};
	}
	if (!ret)
		ret = gpu_m4_vm_tree_bind(vm->tree, ops, count);
	for (u32 i = 0; pa && i < count; i++)
		kvfree(pa[i]);
	kfree(pa);
	kfree(ops);
	gpu_m4_drm_provider_tree_settle_locked(vm, ret);
}

static void gpu_m4_drm_provider_tree_create(struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider *p = vm->provider;
	struct gpu_m4_vm_tree_config config = {
		.ops = &gpu_m4_vm_tree_shadow_ops, .cookie = vm,
		.get = gpu_m4_drm_provider_tree_get, .put = gpu_m4_drm_provider_tree_put,
		.low = p->config.vm_start, .high = p->config.vm_end,
		.kernel_start = vm->kernel_start, .kernel_end = vm->kernel_end,
		.reserved = {
			{ GPU_M4_DRM_UAT_CONTEXT_START,
			  GPU_M4_DRM_UAT_CONTEXT_START + GPU_M4_DRM_UAT_CONTEXT_SIZE },
			{ GPU_M4_DRM_UAT_PRIVATE_START, GPU_M4_DRM_UAT_PRIVATE_END },
		},
		.max_tables = GPU_M4_DRM_UAT_MAX_TABLES,
	};
	struct gpu_m4_vm_tree *tree;

	if (!gpu_m4_vm_tree_mode() || vm->authority)
		return;
	tree = gpu_m4_vm_tree_create(&config);
	if (IS_ERR(tree))
		return;
	mutex_lock(p->lock);
	vm->tree = tree;
	/* The 64 KiB context: the initial version's whole private baseline. */
	if (vm->active && !vm->closed)
		gpu_m4_drm_provider_tree_private_locked(vm, vm->active->private,
							vm->active->private_count);
	mutex_unlock(p->lock);
}

static void gpu_m4_drm_provider_tree_bind_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_vm_change *changes,
	struct gpu_m4_drm_provider_object *const *staged, u32 count)
{
	struct gpu_m4_vm_tree_op *ops;
	int ret;

	lockdep_assert_held(vm->provider->lock);
	if (!gpu_m4_drm_provider_tree_live(vm))
		return;
	ops = kcalloc(count, sizeof(*ops), GFP_KERNEL);
	if (!ops) {
		gpu_m4_drm_provider_tree_settle_locked(vm, -ENOMEM);
		return;
	}
	for (u32 i = 0; i < count; i++) {
		const struct drm_asahi_gem_bind_op *op = &changes[i].op;

		ops[i] = (struct gpu_m4_vm_tree_op) { .va = op->addr, .size = op->range };
		if (op->flags & DRM_ASAHI_BIND_UNBIND) {
			ops[i].flags = GPU_M4_VM_TREE_UNBIND;
			continue;
		}
		ops[i].object = staged[i];
		ops[i].pa = staged[i]->backing.pa;
		ops[i].object_size = staged[i]->backing.size;
		ops[i].identity = staged[i]->backing.identity;
		ops[i].cpu_policy = staged[i]->backing.cpu_policy;
		ops[i].offset = op->offset;
		ops[i].access = op->flags & (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE);
		if (op->flags & DRM_ASAHI_BIND_SINGLE_PAGE)
			ops[i].flags = GPU_M4_VM_TREE_SINGLE_PAGE;
	}
	ret = gpu_m4_vm_tree_bind(vm->tree, ops, count);
	kfree(ops);
	gpu_m4_drm_provider_tree_settle_locked(vm, ret);
}

static void gpu_m4_drm_provider_tree_close_locked(struct gpu_m4_drm_provider_vm *vm,
	struct drm_gem_object *object)
{
	struct gpu_m4_drm_provider_object *o;

	lockdep_assert_held(vm->provider->lock);
	if (!gpu_m4_drm_provider_tree_live(vm))
		return;
	o = gpu_m4_drm_provider_object_find(vm, object);
	gpu_m4_drm_provider_tree_settle_locked(vm,
		o ? gpu_m4_vm_tree_remove_object(vm->tree, o) : 0);
}

/* Native baseline members as DRIVER extents holding their objects, or (add
 * false) their removal with the full clear/clean/TLBI/DSB SY sequence.
 */
static void gpu_m4_drm_provider_tree_native_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count, bool add)
{
	struct gpu_m4_vm_tree_op *ops;
	int ret;

	if (!vm || !maps || !count)
		return;
	lockdep_assert_held(vm->provider->lock);
	if (vm->closed || !gpu_m4_drm_provider_tree_live(vm))
		return;
	ops = kcalloc(count, sizeof(*ops), GFP_KERNEL);
	if (!ops) {
		gpu_m4_drm_provider_tree_settle_locked(vm, -ENOMEM);
		return;
	}
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_provider_private_gem *m = &maps[i];

		ops[i] = (struct gpu_m4_vm_tree_op) {
			.va = m->va, .size = m->size,
			.flags = GPU_M4_VM_TREE_DRIVER | (add ? 0 : GPU_M4_VM_TREE_UNBIND),
		};
		if (!add)
			continue;
		ops[i].object = m->object;
		ops[i].pa = m->object->backing.pa;
		ops[i].object_size = m->object->backing.size;
		ops[i].identity = m->object->backing.identity;
		ops[i].cpu_policy = m->object->backing.cpu_policy;
		ops[i].offset = m->offset;
		ops[i].access = m->access;
	}
	ret = gpu_m4_vm_tree_bind(vm->tree, ops, count);
	kfree(ops);
	gpu_m4_drm_provider_tree_settle_locked(vm, ret);
}

void gpu_m4_drm_provider_vm_tree_native_add_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count)
{
	gpu_m4_drm_provider_tree_native_locked(vm, maps, count, true);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_tree_native_add_locked);

void gpu_m4_drm_provider_vm_tree_native_remove_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count)
{
	gpu_m4_drm_provider_tree_native_locked(vm, maps, count, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_tree_native_remove_locked);

/* Is v's whole image the VM's current one (user generation and ledger, common
 * private inventory, native baseline), so the tree must equal it?
 */
static bool gpu_m4_drm_provider_tree_current(struct gpu_m4_drm_provider_version *v)
{
	struct gpu_m4_drm_provider_vm *vm = v->vm;
	struct gpu_m4_drm_provider_version *a = vm->active;
	u32 members = v->baseline ? v->baseline->count : 0;

	if (!a || a->baseline || a->private_count != a->base_private_count ||
	    (v->baseline ? v->baseline != vm->baseline : vm->baseline && vm->baseline->count) ||
	    v->base_private_count != a->private_count ||
	    v->private_count != v->base_private_count + members)
		return false;
	if (v == a)
		return true;
	for (u32 i = 0; i < a->private_count; i++)
		if (!gpu_m4_drm_provider_private_same(&v->private[i], &a->private[i]))
			return false;
	return v->ledger.leaf == a->ledger.leaf ||
		gpu_m4_drm_provider_version_same_user_locked(v, a);
}

/* REMOVE-DEBUG mode 2: a new composed version of the current image. */
static void gpu_m4_drm_provider_tree_composed_locked(struct gpu_m4_drm_provider_version *v)
{
	struct gpu_m4_drm_provider_vm *vm = v->vm;

	if (gpu_m4_vm_tree_mode() < 2 || !gpu_m4_drm_provider_tree_live(vm))
		return;
	if (gpu_m4_drm_provider_tree_current(v))
		gpu_m4_drm_provider_tree_oracle_locked(vm, v, v->private_count, NULL);
	else
		gpu_m4_vm_tree_oracle_skipped();
}

static int gpu_m4_drm_provider_tree_graph_word(void *cookie, u64 va, u64 word)
{
	struct gpu_m4_drm_provider_image *image = cookie;
	bool driver = (va >= GPU_M4_DRM_UAT_CONTEXT_START &&
		       va - GPU_M4_DRM_UAT_CONTEXT_START < GPU_M4_DRM_UAT_CONTEXT_SIZE) ||
		(va >= GPU_M4_DRM_UAT_PRIVATE_START && va < GPU_M4_DRM_UAT_PRIVATE_END);

	if (image->count == image->capacity)
		return -E2BIG;
	image->word[image->count++] = (struct gpu_m4_vm_tree_word) {
		.va = va, .pte = word, .driver = driver,
	};
	return 0;
}

/*
 * REMOVE-DEBUG mode 2: a GRAPH version becoming bound (installation begin or
 * adoption). When it is the current image, its actual constructed table
 * pages, decoded, equal the tree's complete image: the version that the
 * hardware translates through, against its future replacement.
 */
static void gpu_m4_drm_provider_tree_bound_locked(struct gpu_m4_drm_provider_version *v)
{
	struct gpu_m4_drm_provider_vm *vm = v->vm;
	struct gpu_m4_drm_provider_image image = { 0 };
	u64 va = 0, capacity;
	int ret;

	if (gpu_m4_vm_tree_mode() < 2 || !gpu_m4_drm_provider_tree_live(vm))
		return;
	if (v->state != GPU_M4_PROVIDER_VERSION_GRAPH || IS_ERR_OR_NULL(v->uat) ||
	    !gpu_m4_drm_provider_tree_current(v)) {
		gpu_m4_vm_tree_oracle_skipped();
		return;
	}
	capacity = gpu_m4_drm_uat_leaves(v->uat);
	image.capacity = min_t(u64, capacity, GPU_M4_DRM_UAT_MAX_LEAVES);
	image.word = kvcalloc(max(image.capacity, 1U), sizeof(*image.word), GFP_KERNEL);
	ret = image.word ? gpu_m4_drm_uat_leaf_words(v->uat, gpu_m4_drm_provider_tree_graph_word,
						     &image) : -ENOMEM;
	if (ret) {
		gpu_m4_vm_tree_oracle_skipped();
	} else {
		ret = gpu_m4_vm_tree_compare_words(vm->tree, image.word, image.count, &va);
		if (ret)
			pr_warn("GPU shadow VM tree differs from the bound graph: generation=%llu va=%#llx error=%d\n",
				v->generation, va, ret);
		if (ret && ret != -ESTALE)
			gpu_m4_vm_tree_desync(vm->tree);
	}
	kvfree(image.word);
}

static bool gpu_m4_drm_provider_reserved(u64 va, u64 size);

/* ---- S3c: the persistent tree as the VM's hardware authority ----
 *
 * vm->authority (vm_tree_authority at creation, ordinary profile): the tree
 * is installed (TTBAT names its root through the VM's TREE version), edited
 * in place with real TLBI, and never desynced. Its callbacks take no lock:
 * a reference is an atomic increment of an already held object, a drop is
 * object_put_deferred (the last one under the mutex in tree_work), and the
 * last guard of a closed GEM/VM kicks tree_work for the deferred removal.
 * Invariants I1-I4 are the tree's (gpu-m4-drm-vm-tree.h); here:
 * - user mappings are only ever in the tree (no version keeps a pin), so a
 *   pin lives exactly as long as an extent (or a deferred CLOSED extent);
 * - the context and globals DRIVER extents rely on the VM's own context
 *   pages and on tree_holds, both released only after the tree is destroyed
 *   at the VM's final free, which needs every TREE version reference (jobs,
 *   residency) and therefore every guard and binding gone first.
 */
static int gpu_m4_drm_provider_tree_ref(void *cookie __always_unused, void *object)
{
	struct gpu_m4_drm_provider_object *o = object;

	/* The caller (VM_BIND stage, native unit, private hold) holds one. */
	return refcount_inc_not_zero(&o->refs) ? 0 : -ENOENT;
}

static void gpu_m4_drm_provider_tree_unref(void *cookie __always_unused, void *object)
{
	gpu_m4_drm_provider_object_put_deferred(object);
}

static void gpu_m4_drm_provider_tree_deferred(void *cookie)
{
	struct gpu_m4_drm_provider_vm *vm = cookie;

	gpu_m4_drm_provider_tree_kick(vm->provider);
}

/*
 * Deferred GEM/VM-close removals of every authority VM whose last guard
 * dropped, then the final puts of objects whose last reference was dropped
 * without the mutex, then the ordinary reap. Holds a provider reference
 * taken by the kick.
 */
static void gpu_m4_drm_provider_tree_work(struct work_struct *work)
{
	struct gpu_m4_drm_provider *p = container_of(work, struct gpu_m4_drm_provider, tree_work);
	struct gpu_m4_drm_provider_object *o;
	struct gpu_m4_drm_provider_vm *vm;
	unsigned long flags;
	LIST_HEAD(dead);

	gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_WORK, 1);
	mutex_lock(p->lock);
	list_for_each_entry(vm, &p->vms, link)
		if (vm->authority && vm->tree && !gpu_m4_vm_tree_guards(vm->tree))
			gpu_m4_vm_tree_settle_deferred(vm->tree);
	spin_lock_irqsave(&p->dead_lock, flags);
	list_splice_init(&p->dead_objects, &dead);
	spin_unlock_irqrestore(&p->dead_lock, flags);
	while (!list_empty(&dead)) {
		o = list_first_entry(&dead, struct gpu_m4_drm_provider_object, dead);
		list_del_init(&o->dead);
		gpu_m4_drm_provider_object_put_locked(o);
	}
	mutex_unlock(p->lock);
	gpu_m4_drm_provider_reap(p);
	gpu_m4_drm_provider_put(p);
}

/* The VM's root as TREE version roots: root PA | valid, upper word zero. */
static int gpu_m4_drm_provider_tree_roots(struct gpu_m4_drm_provider_version *v, u64 *low,
	u64 *high)
{
	struct gpu_m4_vm_tree *tree = v->vm->tree;
	int ret = gpu_m4_vm_tree_healthy(tree);

	if (ret)
		return ret;
	*low = gpu_m4_vm_tree_root(tree) | 1;
	*high = 0;
	return 0;
}

static bool gpu_m4_drm_provider_version_tree(const struct gpu_m4_drm_provider_version *v)
{
	return v && v->state == GPU_M4_PROVIDER_VERSION_TREE;
}

/* The TREE version: generation 1 and an empty snapshot forever. */
static int gpu_m4_drm_provider_tree_version_create_locked(struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider *p = vm->provider;
	struct gpu_m4_drm_provider_version *v;
	struct gpu_m4_drm_vm_owner *snapshot;
	int ret;

	lockdep_assert_held(p->lock);
	v = gpu_m4_drm_provider_version_alloc(vm, 0, 0, 1);
	if (IS_ERR(v))
		return PTR_ERR(v);
	v->generation = 1;
	v->private_count = v->base_private_count = 0;
	snapshot = gpu_m4_drm_vm_owner_create_indexed(vm->frontend, p->lock, &v->ledger, NULL,
		v->generation, v, gpu_m4_drm_provider_lookup, NULL, 0);
	if (IS_ERR(snapshot)) {
		gpu_m4_drm_provider_version_put_locked(v);
		return PTR_ERR(snapshot);
	}
	v->snapshot = snapshot;
	v->lineage = snapshot;
	ret = gpu_m4_drm_vm_owner_get_unpublished(snapshot);
	if (ret) {
		v->lineage = NULL;
		gpu_m4_drm_provider_version_put_locked(v);
		return ret;
	}
	v->state = GPU_M4_PROVIDER_VERSION_TREE;
	spin_lock(&vm->object_lock);
	vm->tree_version = v;
	spin_unlock(&vm->object_lock);
	return 0;
}

/* Create the authority tree with the context DRIVER extent and the TREE
 * version. Called once by create (outside the mutex); any failure fails the
 * VM create.
 */
static int gpu_m4_drm_provider_tree_authority_create(struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider *p = vm->provider;
	struct gpu_m4_vm_tree_config config = {
		.ops = &gpu_m4_vm_tree_installed_ops, .cookie = vm,
		.get = gpu_m4_drm_provider_tree_ref, .put = gpu_m4_drm_provider_tree_unref,
		.low = p->config.vm_start, .high = p->config.vm_end,
		.kernel_start = vm->kernel_start, .kernel_end = vm->kernel_end,
		.reserved = {
			{ GPU_M4_DRM_UAT_CONTEXT_START,
			  GPU_M4_DRM_UAT_CONTEXT_START + GPU_M4_DRM_UAT_CONTEXT_SIZE },
			{ GPU_M4_DRM_UAT_PRIVATE_START, GPU_M4_DRM_UAT_PRIVATE_END },
		},
		.max_tables = GPU_M4_DRM_UAT_MAX_TABLES,
		.authority = true, .deferred = gpu_m4_drm_provider_tree_deferred,
	};
	u64 pa[GPU_M4_DRM_UAT_CONTEXT_SIZE / GPU_M4_DRM_PAGE_SIZE];
	struct gpu_m4_vm_tree_op op;
	struct gpu_m4_vm_tree *tree;
	int ret = 0;

	lockdep_assert_not_held(p->lock);
	tree = gpu_m4_vm_tree_create(&config);
	if (IS_ERR(tree))
		return PTR_ERR(tree);
	mutex_lock(p->lock);
	vm->tree = tree;
	/* The 64 KiB context, resolved through the version builder's callback. */
	for (u32 i = 0; !ret && i < ARRAY_SIZE(pa); i++)
		ret = gpu_m4_drm_provider_context_page(&vm->context,
			(u64)i * GPU_M4_DRM_PAGE_SIZE, &pa[i]);
	if (!ret) {
		op = (struct gpu_m4_vm_tree_op) {
			.va = GPU_M4_DRM_UAT_CONTEXT_START, .size = GPU_M4_DRM_UAT_CONTEXT_SIZE,
			.object_size = GPU_M4_DRM_UAT_CONTEXT_SIZE, .pa = pa,
			.access = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE,
			.flags = GPU_M4_VM_TREE_DRIVER, .identity = &vm->context,
			.cpu_policy = GPU_M4_DRM_CPU_NC,
		};
		ret = gpu_m4_vm_tree_bind(tree, &op, 1);
	}
	if (!ret)
		ret = gpu_m4_drm_provider_tree_version_create_locked(vm);
	mutex_unlock(p->lock);
	if (!ret)
		gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_VMS, 1);
	return ret;
}

/*
 * Globals of initialize_private as DRIVER extents of the authority tree,
 * each holding its private mapping's own reference (tree_holds) until the
 * tree is destroyed. Mutex held (the mapping's get needs it).
 */
static int gpu_m4_drm_provider_tree_authority_private_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_uat_private *private, u32 count)
{
	struct gpu_m4_drm_uat_private *holds;
	struct gpu_m4_vm_tree_op *ops;
	u64 **pa;
	u32 held = 0;
	int ret = 0;

	lockdep_assert_held(vm->provider->lock);
	if (vm->tree_holds)
		return -EBUSY;
	holds = kcalloc(count, sizeof(*holds), GFP_KERNEL);
	ops = kcalloc(count, sizeof(*ops), GFP_KERNEL);
	pa = kcalloc(count, sizeof(*pa), GFP_KERNEL);
	if (!holds || !ops || !pa)
		ret = -ENOMEM;
	for (u32 i = 0; !ret && i < count; i++) {
		const struct gpu_m4_drm_uat_private *m = &private[i];
		u64 pages = m->size / GPU_M4_DRM_PAGE_SIZE;

		pa[i] = pages && pages <= GPU_M4_DRM_UAT_MAX_LEAVES ?
			kvcalloc(pages, sizeof(**pa), GFP_KERNEL) : NULL;
		if (!pa[i]) {
			ret = pages ? -ENOMEM : -EINVAL;
			break;
		}
		/* Hold first: pages resolve only through a held mapping. */
		ret = m->get(m->cookie, vm->frontend, m->identity);
		if (ret)
			break;
		holds[held++] = *m;
		for (u64 j = 0; !ret && j < pages; j++)
			ret = m->page(m->cookie, m->offset + j * GPU_M4_DRM_PAGE_SIZE, &pa[i][j]);
		if (ret)
			break;
		ops[i] = (struct gpu_m4_vm_tree_op) {
			.va = m->va, .size = m->size, .object_size = m->size, .pa = pa[i],
			.access = m->access, .flags = GPU_M4_VM_TREE_DRIVER,
			.identity = m->identity, .cpu_policy = m->cpu_policy,
		};
	}
	if (!ret)
		ret = gpu_m4_vm_tree_bind(vm->tree, ops, count);
	for (u32 i = 0; pa && i < count; i++)
		kvfree(pa[i]);
	kfree(pa);
	kfree(ops);
	/* Held even on failure: the VM is closed by the caller and its final
	 * free puts these after the tree (outside every lock).
	 */
	vm->tree_holds = holds;
	vm->tree_hold_count = held;
	return ret;
}

/* Authority VM close: no new guard or user bind; user extents removed now
 * (none guarded) or when the last guard drops; DRIVER extents and the tree
 * stay until the final free. The TREE version reference of the VM goes.
 */
static void gpu_m4_drm_provider_tree_authority_close_locked(struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider_version *v;
	int ret;

	lockdep_assert_held(vm->provider->lock);
	ret = vm->tree ? gpu_m4_vm_tree_close(vm->tree) : 0;
	if (ret) {
		vm->error = vm->error ?: ret;
		gpu_m4_drm_vm_error(vm->frontend, ret);
	}
	spin_lock(&vm->object_lock);
	v = vm->tree_version;
	vm->tree_version = NULL;
	spin_unlock(&vm->object_lock);
	if (v)
		WARN_ON(gpu_m4_drm_provider_version_put_locked(v));
}

/* Final free (outside every lock): tree, then the holds, then the caller
 * frees the context. A refused destroy (a guarded, resident or faulted tree,
 * also one faulted by its final clear) keeps the tree, the holds and so the
 * whole VM: the caller must retain it.
 */
static int gpu_m4_drm_provider_tree_authority_free(struct gpu_m4_drm_provider_vm *vm)
{
	int ret;

	if (!vm->authority)
		return 0;
	ret = gpu_m4_vm_tree_destroy(vm->tree);
	if (ret)
		return ret;
	vm->tree = NULL;
	for (u32 i = 0; i < vm->tree_hold_count; i++)
		vm->tree_holds[i].put(vm->tree_holds[i].cookie);
	kfree(vm->tree_holds);
	vm->tree_holds = NULL;
	return 0;
}

bool gpu_m4_drm_provider_version_live(const struct gpu_m4_drm_provider_version *v)
{
	return gpu_m4_drm_provider_version_tree(v);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_live);

bool gpu_m4_drm_provider_vm_authority(struct gpu_m4_drm_provider_vm *vm)
{
	return vm && vm->authority;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_authority);

struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_vm_tree_version(
	struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider_version *v;

	if (!vm || !vm->authority)
		return ERR_PTR(-EOPNOTSUPP);
	spin_lock(&vm->object_lock);
	v = vm->tree_version;
	if (v && !refcount_inc_not_zero(&v->refs))
		v = NULL;
	spin_unlock(&vm->object_lock);
	return v ?: ERR_PTR(-ENOENT);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_tree_version);

int gpu_m4_drm_provider_vm_tree_capture(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_version **version, struct gpu_m4_drm_vm_owner **owner)
{
	struct gpu_m4_drm_provider_version *v;
	struct gpu_m4_drm_vm_owner *clone;

	if (!version || !owner)
		return -EINVAL;
	*version = NULL;
	*owner = NULL;
	v = gpu_m4_drm_provider_vm_tree_version(vm);
	if (IS_ERR(v))
		return PTR_ERR(v);
	/* The empty, immutable TREE snapshot: a job-local wrapper (no leaf,
	 * row or pin), so stage/timestamp rows join it like any capture.
	 */
	clone = gpu_m4_drm_vm_owner_clone_with_extras(v->snapshot, NULL, 0);
	if (IS_ERR(clone)) {
		mutex_lock(vm->provider->lock);
		WARN_ON(gpu_m4_drm_provider_version_put_locked(v));
		mutex_unlock(vm->provider->lock);
		return PTR_ERR(clone);
	}
	*version = v;
	*owner = clone;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_tree_capture);

bool gpu_m4_drm_provider_vm_object_mapped(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_object *object, u64 offset, u64 size, u32 access)
{
	return vm && vm->authority && object && object->vm == vm &&
		gpu_m4_vm_tree_object_access(vm->tree, object, offset, size, access);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_object_mapped);

int gpu_m4_drm_provider_vm_guard_get(struct gpu_m4_drm_provider_vm *vm)
{
	if (!vm || !vm->authority)
		return -EOPNOTSUPP;
	if (READ_ONCE(vm->error) || READ_ONCE(vm->provider->error))
		return READ_ONCE(vm->provider->error) ?: READ_ONCE(vm->error);
	return gpu_m4_vm_tree_guard_get(vm->tree);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_guard_get);

void gpu_m4_drm_provider_vm_guard_put(struct gpu_m4_drm_provider_vm *vm)
{
	if (vm && vm->authority)
		gpu_m4_vm_tree_guard_put(vm->tree);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_guard_put);

int gpu_m4_drm_provider_vm_covers(struct gpu_m4_drm_provider_vm *vm, u64 va, u64 size,
	u32 access, struct gpu_m4_vm_tree_span *span)
{
	if (!vm || !vm->authority)
		return -EOPNOTSUPP;
	return gpu_m4_vm_tree_covers(vm->tree, va, size, access, span);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_covers);

void gpu_m4_drm_provider_vm_set_asid_locked(struct gpu_m4_drm_provider_vm *vm, u32 asid)
{
	if (!vm)
		return;
	lockdep_assert_held(vm->provider->lock);
	if (vm->authority)
		gpu_m4_vm_tree_set_asid(vm->tree, asid);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_set_asid_locked);

/* Native unit maps as DRIVER extents holding their objects (add), or their
 * removal with the full clear/clean/TLBI/DSB SY sequence. The tree is the
 * authority: failure is returned, nothing desyncs. A removal of maps no
 * longer present (closed VM detached them) is a no-op success.
 */
static int gpu_m4_drm_provider_tree_authority_native_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count, bool add)
{
	struct gpu_m4_drm_vm_leaf leaf;
	struct gpu_m4_vm_tree_op *ops;
	u32 n = 0;
	int ret;

	lockdep_assert_held(vm->provider->lock);
	if (!vm->authority || !maps || !count)
		return -EINVAL;
	if (add && (vm->closed || vm->error))
		return vm->error ?: -ENOENT;
	ops = kcalloc(count, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_provider_private_gem *m = &maps[i];

		if (!add) {
			if (!gpu_m4_vm_tree_maps(vm->tree, m->va, m->size, m->object, m->offset,
						 m->access, true))
				continue;
			ops[n++] = (struct gpu_m4_vm_tree_op) {
				.va = m->va, .size = m->size,
				.flags = GPU_M4_VM_TREE_DRIVER | GPU_M4_VM_TREE_UNBIND,
			};
			continue;
		}
		if (!gpu_m4_drm_provider_object_origin(m->object, vm)) {
			kfree(ops);
			return -EPERM;
		}
		/* A unit's claims are exclusive: an existing mapping at either end
		 * is a duplicate add, never a replacement of another owner.
		 */
		if (gpu_m4_vm_tree_page(vm->tree, m->va, DRM_ASAHI_BIND_READ, &leaf) != -ENOENT ||
		    gpu_m4_vm_tree_page(vm->tree, m->va + m->size - GPU_M4_DRM_PAGE_SIZE,
					DRM_ASAHI_BIND_READ, &leaf) != -ENOENT) {
			kfree(ops);
			return -EBUSY;
		}
		ops[n++] = (struct gpu_m4_vm_tree_op) {
			.va = m->va, .size = m->size, .offset = m->offset,
			.object_size = m->object->backing.size, .object = m->object,
			.pa = m->object->backing.pa, .access = m->access,
			.flags = GPU_M4_VM_TREE_DRIVER, .identity = m->object->backing.identity,
			.cpu_policy = m->object->backing.cpu_policy,
		};
	}
	ret = n ? gpu_m4_vm_tree_bind(vm->tree, ops, n) : 0;
	kfree(ops);
	return ret;
}

int gpu_m4_drm_provider_vm_native_map_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count)
{
	return gpu_m4_drm_provider_tree_authority_native_locked(vm, maps, count, true);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_native_map_locked);

int gpu_m4_drm_provider_vm_native_unmap_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count)
{
	return gpu_m4_drm_provider_tree_authority_native_locked(vm, maps, count, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_native_unmap_locked);

/* Every map is exactly one live DRIVER extent of its object (stage check). */
bool gpu_m4_drm_provider_vm_native_mapped(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count)
{
	if (!vm || !vm->authority || (count && !maps))
		return false;
	for (u32 i = 0; i < count; i++)
		if (!gpu_m4_vm_tree_maps(vm->tree, maps[i].va, maps[i].size, maps[i].object,
					 maps[i].offset, maps[i].access, true))
			return false;
	return true;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_native_mapped);

/*
 * Authority VM_BIND: the whole batch is one tree transaction (all or
 * nothing). No version, plan or commit, and no provider (GPU) mutex for
 * objects this VM already admitted: their registered pin is found under the
 * VM's object_lock. Only the first admission of a GEM into the VM (its page
 * oracle and registration) takes the mutex, inside object_pin_owned.
 */
static int gpu_m4_drm_provider_vm_bind_tree(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_bind *binds, u32 count, bool internal_native, bool defer_reap)
{
	struct gpu_m4_drm_provider *p = vm->provider;
	const u32 known = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE |
		DRM_ASAHI_BIND_SINGLE_PAGE | DRM_ASAHI_BIND_UNBIND;
	struct gpu_m4_drm_provider_object **staged;
	struct gpu_m4_vm_tree_op *ops;
	int ret = 0;

	lockdep_assert_not_held(p->lock);
	staged = kcalloc(count, sizeof(*staged), GFP_KERNEL);
	ops = kcalloc(count, sizeof(*ops), GFP_KERNEL);
	if (!staged || !ops) {
		ret = -ENOMEM;
		goto out;
	}
	if (READ_ONCE(vm->closed) || READ_ONCE(vm->error) || READ_ONCE(p->error)) {
		ret = READ_ONCE(p->error) ?: READ_ONCE(vm->error) ?: -ENOENT;
		goto out;
	}
	for (u32 i = 0; i < count; i++) {
		const struct drm_asahi_gem_bind_op *op = &binds[i].op;
		struct gpu_m4_drm_provider_object *o;

		if ((op->flags & ~known) ||
		    (internal_native && (op->handle || !gpu_m4_drm_vm_leaf_attrs(op->flags)))) {
			ret = -EINVAL;
			goto out;
		}
		if (!op->range || op->addr > U64_MAX - op->range ||
		    gpu_m4_drm_provider_reserved(op->addr, op->range)) {
			ret = -EADDRINUSE;
			goto out;
		}
		ops[i] = (struct gpu_m4_vm_tree_op) { .va = op->addr, .size = op->range };
		if (op->flags & DRM_ASAHI_BIND_UNBIND) {
			if (binds[i].object || (op->flags & ~DRM_ASAHI_BIND_UNBIND)) {
				ret = -EINVAL;
				goto out;
			}
			ops[i].flags = GPU_M4_VM_TREE_UNBIND;
			continue;
		}
		if (!binds[i].object) {
			ret = -EINVAL;
			goto out;
		}
		o = gpu_m4_drm_provider_object_lookup(vm, binds[i].object);
		if (!o) {
			gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_ADMISSIONS, 1);
			o = gpu_m4_drm_provider_object_pin_owned(vm, binds[i].object, defer_reap);
			if (IS_ERR(o)) {
				ret = PTR_ERR(o);
				goto out;
			}
		}
		staged[i] = o;
		ops[i] = (struct gpu_m4_vm_tree_op) {
			.va = op->addr, .size = op->range, .offset = op->offset,
			.object_size = o->backing.size, .object = o, .pa = o->backing.pa,
			.access = op->flags & (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE),
			.flags = op->flags & DRM_ASAHI_BIND_SINGLE_PAGE ? GPU_M4_VM_TREE_SINGLE_PAGE : 0,
			.identity = o->backing.identity, .cpu_policy = o->backing.cpu_policy,
		};
	}
	ret = gpu_m4_vm_tree_bind(vm->tree, ops, count);
out:
	gpu_m4_drm_provider_tree_count(ret ? GPU_M4_DRM_PROVIDER_TREE_BIND_REFUSED :
				       GPU_M4_DRM_PROVIDER_TREE_BINDS, 1);
	if (!ret)
		gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_BIND_OPS, count);
	for (u32 i = 0; staged && i < count; i++)
		gpu_m4_drm_provider_object_put_deferred(staged[i]);
	kfree(staged);
	kfree(ops);
	/* A faulted tree (an inconsistency its edit found) stops this VM. */
	if (ret == -EIO) {
		mutex_lock(p->lock);
		vm->error = vm->error ?: ret;
		gpu_m4_drm_vm_error(vm->frontend, ret);
		if (p->config.error_wake_locked)
			p->config.error_wake_locked(p, vm);
		mutex_unlock(p->lock);
	}
	return ret;
}

/* Authority GEM close: the GEM's user extents now, or CLOSED until the last
 * guard (tree). No provider mutex.
 */
static void gpu_m4_drm_provider_vm_bo_close_tree(struct gpu_m4_drm_provider_vm *vm,
	struct drm_gem_object *object)
{
	struct gpu_m4_drm_provider_object *o;
	int ret;

	if (!object || READ_ONCE(vm->closed))
		return;
	o = gpu_m4_drm_provider_object_lookup(vm, object);
	if (!o)
		return;
	gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_BO_CLOSES, 1);
	ret = gpu_m4_vm_tree_remove_object(vm->tree, o);
	gpu_m4_drm_provider_object_put_deferred(o);
	if (ret) {
		struct gpu_m4_drm_provider *p = vm->provider;

		mutex_lock(p->lock);
		vm->error = vm->error ?: ret;
		gpu_m4_drm_vm_error(vm->frontend, ret);
		if (p->config.error_wake_locked)
			p->config.error_wake_locked(p, vm);
		mutex_unlock(p->lock);
	}
}

static struct gpu_m4_drm_provider_vm *gpu_m4_drm_provider_vm_create_owned(
	struct gpu_m4_drm_provider *p, struct gpu_m4_drm_vm *frontend,
	u64 kernel_start, u64 kernel_end, bool defer_reap)
{
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_provider_version *v;
	int ret;

	if (!p || !frontend || !gpu_m4_drm_aligned(kernel_start) ||
	    !gpu_m4_drm_aligned(kernel_end) || kernel_start >= kernel_end ||
	    kernel_start < p->config.vm_start || kernel_end > p->config.vm_end ||
	    (kernel_start < GPU_M4_DRM_UAT_CONTEXT_START + GPU_M4_DRM_UAT_CONTEXT_SIZE &&
	     kernel_end > GPU_M4_DRM_UAT_CONTEXT_START) ||
	    (kernel_start < GPU_M4_DRM_UAT_PRIVATE_END && kernel_end > GPU_M4_DRM_UAT_PRIVATE_START))
		return ERR_PTR(-EINVAL);
	lockdep_assert_not_held(p->lock);
	vm = kzalloc_obj(*vm);
	if (!vm)
		return ERR_PTR(-ENOMEM);
	vm->provider = p;
	vm->frontend = frontend;
	vm->kernel_start = kernel_start;
	vm->kernel_end = kernel_end;
	/* S3c: ordinary user VMs only; fixture VMs keep their versions. */
	vm->authority = READ_ONCE(vm_tree_authority) && p->config.defer_user_materialization;
	spin_lock_init(&vm->object_lock);
	refcount_set(&vm->refs, 1);
	INIT_LIST_HEAD(&vm->objects);
	INIT_LIST_HEAD(&vm->versions);
	INIT_LIST_HEAD(&vm->private_leases);
	INIT_LIST_HEAD(&vm->garbage);
	spin_lock_init(&vm->capture_lock);
	mutex_lock(p->lock);
	ret = p->error ? p->error : gpu_m4_drm_provider_charge(p,
		GPU_M4_DRM_UAT_CONTEXT_SIZE + sizeof(*vm));
	if (ret) {
		mutex_unlock(p->lock);
		kfree(vm);
		return ERR_PTR(ret);
	}
	gpu_m4_drm_provider_get(p);
	gpu_m4_drm_vm_get(frontend);
	gpu_m4_drm_provider_vm_list_locked(p, vm);
	mutex_unlock(p->lock);
	ret = gpu_m4_drm_provider_context_alloc(vm);
	mutex_lock(p->lock);
	if (!ret && p->error)
		ret = p->error;
	if (ret)
		goto fail;
	v = gpu_m4_drm_provider_version_alloc(vm, 1, 1, 1);
	if (IS_ERR(v)) {
		ret = PTR_ERR(v);
		goto fail;
	}
	v->generation = 1;
	ret = gpu_m4_drm_provider_version_build(v, NULL);
	if (ret) {
		gpu_m4_drm_provider_version_put_locked(v);
		goto fail;
	}
	vm->active = v;
	vm->generation = 1;
	mutex_unlock(p->lock);
	if (vm->authority) {
		ret = gpu_m4_drm_provider_tree_authority_create(vm);
		if (ret) {
			mutex_lock(p->lock);
			gpu_m4_drm_provider_vm_close_locked(vm);
			mutex_unlock(p->lock);
			if (!defer_reap)
				gpu_m4_drm_provider_reap(p);
			return ERR_PTR(ret);
		}
		return vm;
	}
	gpu_m4_drm_provider_tree_create(vm);
	return vm;
fail:
	vm->closed = true;
	gpu_m4_drm_provider_vm_put_locked(vm);
	mutex_unlock(p->lock);
	if (!defer_reap)
		gpu_m4_drm_provider_reap(p);
	return ERR_PTR(ret);
}

struct gpu_m4_drm_provider_vm *gpu_m4_drm_provider_vm_create(
	struct gpu_m4_drm_provider *p, struct gpu_m4_drm_vm *frontend,
	u64 kernel_start, u64 kernel_end)
{
	return gpu_m4_drm_provider_vm_create_owned(p, frontend, kernel_start, kernel_end, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_create);

struct gpu_m4_drm_provider_vm *gpu_m4_drm_provider_vm_create_deferred(
	struct gpu_m4_drm_provider *p, struct gpu_m4_drm_vm *frontend,
	u64 kernel_start, u64 kernel_end)
{
	return gpu_m4_drm_provider_vm_create_owned(p, frontend, kernel_start, kernel_end, true);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_create_deferred);

static bool gpu_m4_drm_provider_reserved(u64 va, u64 size)
{
	return (va < GPU_M4_DRM_UAT_CONTEXT_START + GPU_M4_DRM_UAT_CONTEXT_SIZE &&
		va + size > GPU_M4_DRM_UAT_CONTEXT_START) ||
		(va < GPU_M4_DRM_UAT_PRIVATE_END && va + size > GPU_M4_DRM_UAT_PRIVATE_START);
}

/*
 * Detached candidate for an ordinary bind or a BO close: its user leaves, the
 * retained index derived from the held source index, and the facts the short
 * commit needs. Built only from an exact, held, immutable source version and
 * held staged pins; never under the provider mutex, never touching the source.
 */
struct gpu_m4_drm_provider_plan {
	struct gpu_m4_drm_vm_leaf *leaf;
	struct gpu_m4_drm_vm_ledger_index index;
	const void **identity;
	struct gpu_m4_drm_provider_user_facts facts;
	u64 capacity;
	u32 identities, count;
	bool proven;	/* delta proof passed; new pages still need the page oracle */
};

/* Bounded replanning when another same-VM writer commits during planning. */
#define GPU_M4_DRM_PROVIDER_REPLANS 8

static void gpu_m4_drm_provider_plan_free(struct gpu_m4_drm_provider_plan *plan)
{
	kvfree(plan->leaf);
	gpu_m4_drm_vm_ledger_index_free(&plan->index);
	kvfree(plan->identity);
	memset(plan, 0, sizeof(*plan));
}

static int gpu_m4_drm_provider_plan_facts(struct gpu_m4_drm_provider_plan *plan, u32 retained)
{
	plan->facts.added_from = retained;
	gpu_m4_drm_vm_ledger_prefixes(plan->leaf, plan->index.va, plan->count,
				      &plan->facts.l1, &plan->facts.l2);
	return gpu_m4_drm_vm_ledger_identities(plan->leaf, &plan->index, &plan->identity,
					       &plan->identities);
}

/* Pure: source ledger/index are the caller's by-value copies of a held version. */
static int gpu_m4_drm_provider_plan_bind(struct gpu_m4_drm_provider *p,
	const struct gpu_m4_drm_vm_ledger *source, const struct gpu_m4_drm_vm_ledger_index *index,
	const struct gpu_m4_drm_vm_change *changes, u32 count, bool internal_native,
	struct gpu_m4_drm_provider_plan *plan)
{
	struct gpu_m4_drm_vm_ledger candidate;
	u64 running;
	u32 retained;
	int ret;

	plan->capacity = source->count;
	if (!internal_native) {
		ret = gpu_m4_drm_vm_ledger_plan_capacity_indexed(source, index, changes, count,
			p->config.max_leaves, &plan->capacity);
		if (ret)
			return ret;
	} else {
		running = plan->capacity;
		for (u32 i = 0; i < count; i++) {
			u64 pages = changes[i].op.range / GPU_M4_DRM_PAGE_SIZE;

			if (changes[i].op.flags & DRM_ASAHI_BIND_UNBIND) {
				if (pages > running)
					return -ENOENT;
				running -= pages;
			} else {
				if (pages > p->config.max_leaves - running)
					return -ENOSPC;
				running += pages;
				plan->capacity = max(plan->capacity, running);
			}
		}
	}
	plan->leaf = kvcalloc((plan->capacity ? plan->capacity : 1), sizeof(*plan->leaf), GFP_KERNEL);
	if (!plan->leaf)
		return -ENOMEM;
	if (!gpu_m4_drm_vm_ledger_index_valid(source, index))
		return 0; /* The exact sequential reference decides under the lock. */
	candidate = (struct gpu_m4_drm_vm_ledger) {
		.leaf = plan->leaf, .capacity = (plan->capacity ? plan->capacity : 1),
	};
	ret = gpu_m4_drm_vm_ledger_plan_delta(source, index, &candidate, &plan->index,
					      changes, count, &retained);
	if (ret > 0)
		return 0;
	if (ret)
		return ret;
	plan->count = candidate.count;
	ret = gpu_m4_drm_provider_plan_facts(plan, retained);
	if (!ret)
		plan->proven = true;
	return ret;
}

/* Reverse object->range removal from the held source's retained identity index. */
static int gpu_m4_drm_provider_plan_close(const struct gpu_m4_drm_vm_ledger *source,
	const struct gpu_m4_drm_vm_ledger_index *index, const void *identity,
	struct gpu_m4_drm_provider_plan *plan)
{
	struct gpu_m4_drm_vm_ledger candidate;
	int ret;

	plan->capacity = source->count;
	plan->leaf = kvcalloc(max(source->count, 1U), sizeof(*plan->leaf), GFP_KERNEL);
	if (!plan->leaf)
		return -ENOMEM;
	candidate = (struct gpu_m4_drm_vm_ledger) {
		.leaf = plan->leaf, .capacity = max(source->count, 1U),
	};
	ret = gpu_m4_drm_vm_ledger_filter_identity(source, index, identity, &candidate,
						   &plan->index);
	if (ret)
		return ret;
	plan->count = candidate.count;
	ret = gpu_m4_drm_provider_plan_facts(plan, candidate.count);
	if (!ret)
		plan->proven = true;
	return ret;
}

/*
 * Keep exactly the objects named by the candidate's sorted distinct user
 * identities plus its native-private pins: one merge with the sorted object
 * inventory, not a search per leaf. Stable compaction keeps the inventory
 * sorted.
 */
static int gpu_m4_drm_provider_version_prune_identities(struct gpu_m4_drm_provider_version *v,
	const void *const *identity, u32 count)
{
	u32 kept = 0, at = 0;
	int ret = gpu_m4_drm_provider_version_index(v);

	if (ret)
		return ret;
	memset(v->object_used, 0, v->object_count * sizeof(*v->object_used));
	for (u32 i = 0; i < count; i++) {
		while (at < v->object_count &&
		       (unsigned long)v->objects[at]->backing.identity < (unsigned long)identity[i])
			at++;
		if (at == v->object_count || v->objects[at]->backing.identity != identity[i])
			return -EPROTO;
		v->object_used[at] = true;
	}
	for (u32 i = 0; i < v->native_private_count; i++) {
		const void *object = gpu_m4_drm_bo_pin_object(v->native_private[i].pin);
		u32 first = 0, end = v->object_count;

		while (first < end) {
			u32 middle = first + (end - first) / 2;

			if ((unsigned long)v->objects[middle]->backing.identity < (unsigned long)object)
				first = middle + 1;
			else
				end = middle;
		}
		if (first == v->object_count || v->objects[first]->backing.identity != object)
			return -EPROTO;
		v->object_used[first] = true;
	}
	for (u32 i = 0; i < v->object_count; i++)
		if (v->object_used[i])
			v->objects[kept++] = v->objects[i];
		else
			gpu_m4_drm_provider_object_put_locked(v->objects[i]);
	v->object_count = kept;
	return 0;
}

/*
 * Ordinary bind: pins are staged outside every lock; a short section captures
 * the exact active owner (held) and generation; the candidate ledger/index is
 * planned outside the provider mutex against that held immutable owner; the
 * commit rechecks VM OPEN, generation and owner identity, admits only the new
 * pages through the exact page oracle, and swaps the active owner. A writer
 * that committed meanwhile forces a replan against its owner, never an edit of
 * partially planned state. Anything the delta cannot prove is decided by the
 * unchanged sequential reference under the lock.
 */
static int gpu_m4_drm_provider_vm_bind_owned(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_bind *binds, u32 count, bool internal_native, bool defer_reap)
{
	struct gpu_m4_drm_provider *p;
	struct gpu_m4_drm_provider_object **staged;
	struct gpu_m4_drm_vm_change *changes;
	struct gpu_m4_drm_provider_version *v = NULL, *old = NULL;
	struct gpu_m4_drm_provider_plan plan = { 0 };
	struct gpu_m4_drm_vm_ledger_index index;
	struct gpu_m4_drm_vm_ledger source;
	u64 generation, revision;
	int ret;

	if (!vm || !binds || !count || count > GPU_M4_DRM_MAX_BINDS)
		return -EINVAL;
	if (vm->authority)
		return gpu_m4_drm_provider_vm_bind_tree(vm, binds, count, internal_native, defer_reap);
	p = vm->provider;
	lockdep_assert_not_held(p->lock);
	staged = kcalloc(count, sizeof(*staged), GFP_KERNEL);
	changes = kcalloc(count, sizeof(*changes), GFP_KERNEL);
	if (!staged || !changes) {
		ret = -ENOMEM;
		goto free_arrays;
	}
	mutex_lock(p->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm)) {
		ret = p->error ? p->error : -ENOENT;
		mutex_unlock(p->lock);
		goto free_arrays;
	}
	generation = vm->generation;
	gpu_m4_drm_provider_vm_get_locked(vm);
	mutex_unlock(p->lock);
	/* No provider lock is held while acquiring native GEM reservations. */
	for (u32 i = 0; i < count; i++) {
		const struct drm_asahi_gem_bind_op *op = &binds[i].op;

		if (internal_native && (op->handle || !gpu_m4_drm_vm_leaf_attrs(op->flags))) {
			ret = -EINVAL;
			goto release_stage;
		}
		if (!op->range || op->addr > U64_MAX - op->range ||
		    gpu_m4_drm_provider_reserved(op->addr, op->range)) {
			ret = -EADDRINUSE;
			goto release_stage;
		}
		changes[i].op = *op;
		if (op->flags & DRM_ASAHI_BIND_UNBIND) {
			if (binds[i].object) {
				ret = -EINVAL;
				goto release_stage;
			}
			continue;
		}
		staged[i] = gpu_m4_drm_provider_object_pin_owned(vm, binds[i].object, defer_reap);
		if (IS_ERR(staged[i])) {
			ret = PTR_ERR(staged[i]);
			staged[i] = NULL;
			goto release_stage;
		}
		changes[i].backing = &staged[i]->backing;
		/* object_pin proved actual native frontend/private-VM identity,
		 * immutable policy, retained pages and the session exclusions.
		 */
		changes[i].internal_native = internal_native;
	}
	mutex_lock(p->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm)) {
		ret = p->error ? p->error : -ENOENT;
		goto unlock;
	}
	if (vm->generation != generation || generation == U64_MAX) {
		ret = generation == U64_MAX ? -EOVERFLOW : -EAGAIN;
		goto unlock;
	}
	for (u32 attempt = 0;; attempt++) {
		generation = vm->generation;
		revision = vm->baseline_revision;
		old = vm->active;
		ret = gpu_m4_drm_provider_version_get_locked(old);
		if (ret) {
			old = NULL;
			goto unlock;
		}
		/* By-value views of the held immutable owner for the unlocked plan. */
		source = old->ledger;
		index = gpu_m4_drm_provider_version_indexed(old) ? old->index :
			(struct gpu_m4_drm_vm_ledger_index) { 0 };
		mutex_unlock(p->lock);
		ret = gpu_m4_drm_provider_plan_bind(p, &source, &index, changes, count,
						    internal_native, &plan);
		mutex_lock(p->lock);
		if (!ret && !gpu_m4_drm_provider_vm_open_locked(vm))
			ret = p->error ? p->error : -ENOENT;
		if (ret)
			goto unlock;
		/* The plan carries the exact native baseline it was made against: a
		 * concurrent baseline commit forces a replan, as a racing bind does.
		 */
		if (vm->generation == generation && vm->active == old &&
		    vm->baseline_revision == revision)
			break;
		gpu_m4_drm_provider_version_put_locked(old);
		old = NULL;
		if (vm->generation == U64_MAX || attempt + 1 == GPU_M4_DRM_PROVIDER_REPLANS) {
			ret = vm->generation == U64_MAX ? -EOVERFLOW : -EAGAIN;
			goto unlock;
		}
		mutex_unlock(p->lock);
		gpu_m4_drm_provider_plan_free(&plan);
		mutex_lock(p->lock);
		if (!gpu_m4_drm_provider_vm_open_locked(vm)) {
			ret = p->error ? p->error : -ENOENT;
			goto unlock;
		}
	}
	v = gpu_m4_drm_provider_version_alloc_storage(vm, plan.capacity,
		old->object_count + count, old->private_count, plan.leaf);
	if (IS_ERR(v)) {
		ret = PTR_ERR(v);
		v = NULL;
		goto unlock;
	}
	plan.leaf = NULL;
	ret = gpu_m4_drm_provider_version_copy_meta(v, old);
	if (ret)
		goto unlock;
	for (u32 i = 0; i < count; i++)
		if (staged[i] && !gpu_m4_drm_provider_version_object(v, staged[i]->backing.identity)) {
			v->objects[v->object_count++] = staged[i];
			gpu_m4_drm_provider_object_get_locked(staged[i]);
		}
	ret = gpu_m4_drm_provider_version_index(v);
	if (ret)
		goto unlock;
	if (plan.proven) {
		v->ledger.count = plan.count;
		v->index = plan.index;
		plan.index = (struct gpu_m4_drm_vm_ledger_index) { 0 };
		/* The exact page oracle sees every newly introduced page against
		 * precisely the held old+staged native inventory.
		 */
		if (gpu_m4_drm_vm_ledger_plan_admit(&v->ledger, changes, count))
			plan.proven = false;
	}
	if (!plan.proven) {
		gpu_m4_drm_vm_ledger_index_free(&v->index);
		v->ledger.count = 0;
		source = old->ledger;
		source.owner = v;
		ret = gpu_m4_drm_vm_ledger_plan_indexed(&source, NULL, &v->ledger, &v->index,
							changes, count);
		if (ret)
			goto unlock;
	}
	ret = plan.proven ? gpu_m4_drm_provider_version_prune_identities(v, plan.identity,
									 plan.identities) :
		gpu_m4_drm_provider_version_prune(v);
	if (ret)
		goto unlock;
	v->generation = generation + 1;
	v->baseline_revision = revision;
	ret = p->config.defer_user_materialization && !internal_native ?
		gpu_m4_drm_provider_version_logical(v, old, plan.proven ? &plan.facts : NULL) :
		gpu_m4_drm_provider_version_build(v, NULL);
	if (!ret) {
		gpu_m4_drm_provider_vm_withdraw_locked(vm);
		vm->active = v;
		vm->generation = v->generation;
		gpu_m4_drm_provider_tree_bind_locked(vm, changes, staged, count);
		gpu_m4_drm_provider_version_put_locked(old); /* planning hold */
		gpu_m4_drm_provider_version_put_locked(old); /* former active owner */
		old = NULL;
		v = NULL;
	}
unlock:
	if (v)
		gpu_m4_drm_provider_version_put_locked(v);
	if (old)
		gpu_m4_drm_provider_version_put_locked(old);
	mutex_unlock(p->lock);
	gpu_m4_drm_provider_plan_free(&plan);
release_stage:
	mutex_lock(p->lock);
	for (u32 i = 0; i < count; i++)
		if (staged[i])
			gpu_m4_drm_provider_object_put_locked(staged[i]);
	gpu_m4_drm_provider_vm_put_locked(vm);
	mutex_unlock(p->lock);
	if (!defer_reap)
		gpu_m4_drm_provider_reap(p);
free_arrays:
	kfree(changes);
	kfree(staged);
	return ret;
}

int gpu_m4_drm_provider_vm_bind(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_bind *binds, u32 count)
{
	return gpu_m4_drm_provider_vm_bind_owned(vm, binds, count, false, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_bind);

int gpu_m4_drm_provider_vm_bind_deferred(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_bind *binds, u32 count)
{
	return gpu_m4_drm_provider_vm_bind_owned(vm, binds, count, false, true);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_bind_deferred);

int gpu_m4_drm_provider_vm_bind_native(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_bind *binds, u32 count)
{
	return gpu_m4_drm_provider_vm_bind_owned(vm, binds, count, true, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_bind_native);

void gpu_m4_drm_provider_vm_bo_close(struct gpu_m4_drm_provider_vm *vm,
	struct drm_gem_object *object)
{
	struct gpu_m4_drm_provider_version *old = NULL, *v = NULL;
	struct gpu_m4_drm_provider_plan plan = { 0 };
	struct gpu_m4_drm_vm_ledger_index index;
	struct gpu_m4_drm_vm_ledger source;
	struct gpu_m4_drm_provider *p;
	u64 generation, revision;
	bool held = false;
	int ret = 0;

	if (!vm)
		return;
	if (vm->authority) {
		gpu_m4_drm_provider_vm_bo_close_tree(vm, object);
		return;
	}
	p = vm->provider;
	lockdep_assert_not_held(p->lock);
	mutex_lock(p->lock);
	for (;;) {
		struct gpu_m4_drm_provider_version *active;

		if (!gpu_m4_drm_provider_vm_open_locked(vm))
			goto out;
		active = vm->active;
		if (!object || !active || !refcount_read(&active->refs)) {
			ret = -EINVAL;
			goto fault;
		}
		ret = gpu_m4_drm_vm_ledger_shape(&active->ledger);
		if (ret)
			goto fault;
		if (!gpu_m4_drm_provider_version_indexed(active)) {
			/* An unproven owner is fully validated once and keeps the index. */
			gpu_m4_drm_vm_ledger_index_free(&active->index);
			ret = gpu_m4_drm_vm_ledger_index_create(&active->ledger, &active->index);
			if (ret)
				goto fault;
		}
		/* Reverse object->range lookup instead of a whole-ledger scan. */
		if (!gpu_m4_drm_vm_ledger_index_names(active->ledger.leaf, &active->index, object))
			goto out;
		if (vm->generation == U64_MAX) {
			ret = -EOVERFLOW;
			goto fault;
		}
		if (active->vm != vm || active->generation != vm->generation) {
			ret = -EPROTO;
			goto fault;
		}
		if (!p->config.defer_user_materialization) {
			ret = gpu_m4_drm_uat_verify(active->uat);
			if (ret)
				goto fault;
		}
		ret = gpu_m4_drm_provider_version_get_locked(active);
		if (ret)
			goto fault;
		old = active;
		held = true;
		generation = vm->generation;
		revision = vm->baseline_revision;
		source = old->ledger;
		index = old->index;
		mutex_unlock(p->lock);
		ret = gpu_m4_drm_provider_plan_close(&source, &index, object, &plan);
		mutex_lock(p->lock);
		if (ret)
			goto fault;
		if (!gpu_m4_drm_provider_vm_open_locked(vm))
			goto out;
		/* Exact captured native baseline too: a racing commit replans. */
		if (vm->generation == generation && vm->active == old &&
		    vm->baseline_revision == revision)
			break;
		/* A same-VM writer committed meanwhile: remove from its owner. */
		gpu_m4_drm_provider_version_put_locked(old);
		old = NULL;
		held = false;
		mutex_unlock(p->lock);
		gpu_m4_drm_provider_plan_free(&plan);
		mutex_lock(p->lock);
	}
	/* Removal-only replacement: the filtered successor (same capacity and
	 * inventory bound, a subset of the leaves) never grows the provider, so
	 * a full budget must not refuse it. Its charges may use the old active
	 * version's own footprint, refunded once its last holder drops it.
	 */
	p->replacing = old->metadata_bytes + old->table_bytes;
	v = gpu_m4_drm_provider_version_alloc_storage(vm, old->ledger.count,
		old->object_count, old->private_count, plan.leaf);
	if (IS_ERR(v)) {
		ret = PTR_ERR(v);
		v = NULL;
		goto fault;
	}
	plan.leaf = NULL;
	ret = gpu_m4_drm_provider_version_copy_meta(v, old);
	if (ret)
		goto fault;
	v->ledger.count = plan.count;
	v->index = plan.index;
	plan.index = (struct gpu_m4_drm_vm_ledger_index) { 0 };
	ret = gpu_m4_drm_provider_version_prune_identities(v, plan.identity, plan.identities);
	if (ret)
		goto fault;
	v->generation = vm->generation + 1;
	v->baseline_revision = revision;
	/* A new user generation must NOT clone the old closed-GEM cache/resv set.
	 * Recreate from filtered leaves and the complete retained private extras.
	 */
	ret = p->config.defer_user_materialization ?
		gpu_m4_drm_provider_version_logical(v, old, &plan.facts) :
		gpu_m4_drm_provider_version_build(v, NULL);
	p->replacing = 0;
	if (!ret) {
		gpu_m4_drm_provider_version_put_locked(old); /* planning hold */
		held = false;
		ret = gpu_m4_drm_provider_version_put_locked(old);
	}
	if (ret)
		goto fault;
	gpu_m4_drm_provider_vm_withdraw_locked(vm);
	vm->active = v;
	vm->generation = v->generation;
	v = NULL;
	gpu_m4_drm_provider_tree_close_locked(vm, object);
	goto out;
fault:
	p->replacing = 0;
	if (v)
		gpu_m4_drm_provider_version_put_locked(v);
	if (gpu_m4_drm_provider_capacity_error(ret)) {
		/* Host memory refused only this removal: nothing was published or
		 * changed and every owner (the stale alias included) stays held.
		 * Fail-stop exactly this VM, never the provider or other clients:
		 * it accepts no further update, capture or submission, so the
		 * alias can serve no future work.
		 */
		gpu_m4_drm_provider_vm_withdraw_locked(vm);
		vm->error = ret;
		gpu_m4_drm_vm_error(vm->frontend, ret);
		if (p->config.error_wake_locked)
			p->config.error_wake_locked(p, vm);
		/* REMOVE-DEBUG: a VM-local capacity refusal, not a provider fault. */
		pr_warn("GPU provider capacity: VM-local BO close failure generation=%llu error=%d\n",
			vm->generation, ret);
	} else {
		gpu_m4_drm_provider_fault_locked(p, ret);
	}
out:
	if (held)
		gpu_m4_drm_provider_version_put_locked(old);
	mutex_unlock(p->lock);
	gpu_m4_drm_provider_plan_free(&plan);
	/* Frontend serialization is still held by the callback's caller. Reap
	 * must be scheduled/run separately after ALL destructor-reentered locks.
	 */
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_bo_close);

static int gpu_m4_drm_provider_version_identity_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_vm *expected_vm,
	struct gpu_m4_drm_provider_version *candidate, bool require_open)
{
	struct gpu_m4_drm_provider_vm *found_vm = NULL, *version_vm;

	if (IS_ERR_OR_NULL(p) || IS_ERR_OR_NULL(expected_vm) || IS_ERR_OR_NULL(candidate))
		return -EINVAL;
	lockdep_assert_held(p->lock);
	if (!refcount_read(&p->refs))
		return -ENOENT;
	if (p->error)
		return p->error;
	/* Neither supplied VM nor version can be dereferenced before membership.
	 * Search the provider's retained owners, not caller-controlled lists.
	 */
	version_vm = gpu_m4_drm_provider_version_member(p, candidate);
	if (gpu_m4_drm_provider_vm_member(p, expected_vm))
		found_vm = expected_vm;
	if (!found_vm || !version_vm || !refcount_read(&found_vm->refs) ||
	    !refcount_read(&version_vm->refs) || !refcount_read(&candidate->refs))
		return -ENOENT;
	if (version_vm != found_vm || found_vm->provider != p || candidate->vm != found_vm)
		return -EPERM;
	if (require_open && !gpu_m4_drm_provider_vm_open_locked(found_vm))
		return p->error ? p->error : -ENOENT;
	/* A RETIRED composed record keeps exactly this identity and snapshot;
	 * every image consumer separately requires the GRAPH state.
	 */
	if (IS_ERR_OR_NULL(found_vm->frontend) || !candidate->generation ||
	    candidate->state < GPU_M4_PROVIDER_VERSION_LOGICAL ||
	    (candidate->state < GPU_M4_PROVIDER_VERSION_GRAPH &&
	     candidate->private_held != candidate->private_count) ||
	    (candidate->state >= GPU_M4_PROVIDER_VERSION_SNAPSHOT &&
	     (IS_ERR_OR_NULL(candidate->snapshot) ||
	      gpu_m4_drm_vm_owner_vm(candidate->snapshot) != found_vm->frontend ||
	      gpu_m4_drm_vm_owner_generation(candidate->snapshot) != candidate->generation)) ||
	    (candidate->state == GPU_M4_PROVIDER_VERSION_GRAPH && IS_ERR_OR_NULL(candidate->uat)))
		return -ESTALE;
	return 0;
}

int gpu_m4_drm_provider_version_check_origin_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_vm *expected_vm,
	struct gpu_m4_drm_provider_version *candidate)
{
	return gpu_m4_drm_provider_version_identity_locked(p, expected_vm, candidate, true);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_check_origin_locked);

/* Read-only reclamation precheck for one EXACT retained version, including a
 * closed VM's captured history: registered membership precedes dereference,
 * but no open VM is required. Only the real bind-complete/unbind transitions
 * remove residency and protection; this check never clears them. A pending
 * installation, the resident root, a protected or a working version is busy.
 * No refs, flags, roots or payloads change.
 */
int gpu_m4_drm_provider_version_quiescent_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_vm *expected_vm,
	struct gpu_m4_drm_provider_version *candidate)
{
	int ret = gpu_m4_drm_provider_version_identity_locked(p, expected_vm, candidate, false);

	if (ret)
		return ret;
	/* S3c: the TREE version is resident and working for every job of its
	 * VM. A unit's translations end with its tree removal (TLBI), not with
	 * this handle's residency; its own job's receipt proves its use ended.
	 */
	if (gpu_m4_drm_provider_version_tree(candidate))
		return 0;
	if (candidate->binding_asid || candidate->protected || candidate->working)
		return -EBUSY;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_quiescent_locked);

static int gpu_m4_drm_provider_logical_page_locked(struct gpu_m4_drm_provider_version *v,
	u64 va, u32 access)
{
	/* The retained VA index answers in O(log N); an unindexed ledger scans. */
	bool indexed = gpu_m4_drm_provider_version_indexed(v);
	u32 at = indexed ? gpu_m4_drm_vm_ledger_index_find(v->ledger.leaf, &v->index, va) : 0;

	for (u32 i = indexed ? at : 0; i < v->ledger.count; i++) {
		const struct gpu_m4_drm_vm_leaf *leaf = &v->ledger.leaf[i];

		if (leaf->va != va) {
			if (indexed)
				break;
			continue;
		}
		if (access & ~leaf->access)
			return -EACCES;
		return gpu_m4_drm_provider_page_valid(v, leaf->identity, leaf->object_offset, leaf->pa);
	}
	if (v->private_held != v->private_count)
		return -ESTALE;
	for (u32 i = 0; i < v->private_count; i++) {
		const struct gpu_m4_drm_uat_private *m = &v->private[i];
		u64 pa;
		int ret;

		if (va < m->va || va - m->va >= m->size)
			continue;
		if (access & ~m->access)
			return -EACCES;
		ret = m->page(m->cookie, m->offset + va - m->va, &pa);
		return ret ?: gpu_m4_drm_provider_private_ram(pa);
	}
	return -ENOENT;
}

/* Identity, access and bound gates shared by every retained range proof. */
static int gpu_m4_drm_provider_version_span_ready(struct gpu_m4_drm_provider_version *v,
	u64 va, u64 size, u32 access)
{
	struct gpu_m4_drm_provider *p;
	u64 end, first, pages;
	int ret;

	/* The caller already retains this checked exact version. Closing its VM
	 * stops new admissions, not range proofs for already captured work. Raw
	 * identities must enter through check_origin_locked() while OPEN first.
	 */
	if (IS_ERR_OR_NULL(v))
		return -EINVAL;
	p = v->vm->provider;
	lockdep_assert_held(gpu_m4_drm_provider_lock(p));
	ret = gpu_m4_drm_provider_version_identity_locked(p, v->vm, v, false);
	if (ret) {
		/* REMOVE-DEBUG: the retained caller already owns v; record the
		 * obtained failure only, without another validation or payload read.
		 */
		pr_warn("GPU retained range: phase=identity va=%#llx size=%#llx access=%#x generation=%llu state=%u vm_closed=%u error=%d\n",
			(unsigned long long)va, (unsigned long long)size, access,
			(unsigned long long)v->generation, v->state, v->vm->closed, ret);
		return ret;
	}
	if (!access || (access & ~(DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE)) ||
	    !gpu_m4_drm_range(va, size, p->config.vm_start, p->config.vm_end))
		return -EINVAL;
	end = va + size;
	first = va & ~(GPU_M4_DRM_PAGE_SIZE - 1);
	pages = (end - first - 1) / GPU_M4_DRM_PAGE_SIZE + 1;
	if (pages > (u64)p->config.max_leaves + 4)
		return -E2BIG;
	return 0;
}

int gpu_m4_drm_provider_version_range_locked(struct gpu_m4_drm_provider_version *v,
	u64 va, u64 size, u32 access)
{
	struct gpu_m4_drm_vm_leaf leaf;
	u64 end, first;
	int ret = gpu_m4_drm_provider_version_span_ready(v, va, size, access);

	if (ret)
		return ret;
	/* S3c: the live tree's leaves (any extent: native/driver included). */
	if (gpu_m4_drm_provider_version_tree(v))
		return gpu_m4_vm_tree_walk(v->vm->tree, va, size, access, NULL, NULL);
	/* An installed graph: one indexed lookup per manifest block for the run,
	 * then consecutive leaves; the per-page loop below only names the page
	 * of a failure (REMOVE-DEBUG attribution) or proves a logical version.
	 */
	if (v->state == GPU_M4_PROVIDER_VERSION_GRAPH) {
		ret = gpu_m4_drm_uat_range(v->uat, va, size, access, NULL, NULL);
		if (!ret)
			return 0;
	}
	end = va + size;
	first = va & ~(GPU_M4_DRM_PAGE_SIZE - 1);
	for (u64 page = first; page < end; page += GPU_M4_DRM_PAGE_SIZE) {
		int failed = v->state == GPU_M4_PROVIDER_VERSION_GRAPH ?
			gpu_m4_drm_provider_version_page_locked(v, page, access, &leaf) :
			gpu_m4_drm_provider_logical_page_locked(v, page, access);

		if (failed) {
			pr_warn("GPU retained range: phase=page va=%#llx size=%#llx page=%#llx access=%#x generation=%llu state=%u vm_closed=%u error=%d\n",
				(unsigned long long)va, (unsigned long long)size,
				(unsigned long long)page, access,
				(unsigned long long)v->generation, v->state, v->vm->closed, failed);
			return failed;
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_range_locked);

int gpu_m4_drm_provider_version_walk_locked(struct gpu_m4_drm_provider_version *v,
	u64 va, u64 size, u32 access, gpu_m4_drm_uat_leaf_fn fn, void *cookie)
{
	int ret = gpu_m4_drm_provider_version_span_ready(v, va, size, access);

	if (ret)
		return ret;
	if (gpu_m4_drm_provider_version_tree(v))
		return gpu_m4_vm_tree_walk(v->vm->tree, va, size, access, fn, cookie);
	if (v->state != GPU_M4_PROVIDER_VERSION_GRAPH)
		return -ENODATA;
	return gpu_m4_drm_uat_range(v->uat, va, size, access, fn, cookie);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_walk_locked);

#define GPU_M4_PROVIDER_TREE_MEMO 8192U
struct gpu_m4_provider_tree_memo {
	struct gpu_m4_drm_uat_span_key key;
	u64 revision; /* 0: empty (a live tree's revision is never 0 here) */
};

static bool tree_span_memo = true;
module_param(tree_span_memo, bool, 0644);
MODULE_PARM_DESC(tree_span_memo, "Reuse TREE-version span proofs while the VM tree revision is unchanged (default 1)");

static u32 gpu_m4_provider_tree_memo_slot(const struct gpu_m4_drm_uat_span_key *k)
{
	u64 h = (u64)(uintptr_t)k->identity ^ ((u64)(uintptr_t)k->aux << 7) ^ (k->va >> 14) ^
		(k->size >> 11) ^ (k->offset >> 9) ^ k->aux_value ^ ((u64)k->access << 3) ^ k->kind;

	h ^= h >> 29;
	h *= 0x9e3779b97f4a7c15ULL;
	return (h >> 40) % GPU_M4_PROVIDER_TREE_MEMO;
}

static bool gpu_m4_provider_tree_memo_same(const struct gpu_m4_drm_uat_span_key *a,
	const struct gpu_m4_drm_uat_span_key *b)
{
	return a->identity == b->identity && a->aux == b->aux && a->va == b->va &&
		a->size == b->size && a->offset == b->offset && a->aux_value == b->aux_value &&
		a->access == b->access && a->kind == b->kind;
}

static bool gpu_m4_provider_tree_memo_find(struct gpu_m4_drm_provider_version *v,
	const struct gpu_m4_drm_uat_span_key *key)
{
	struct gpu_m4_drm_provider_vm *vm = v->vm;
	u64 revision = gpu_m4_vm_tree_revision(vm->tree);
	struct gpu_m4_provider_tree_memo *m;

	vm->tree_memo_pending = revision;
	if (!READ_ONCE(tree_span_memo) || !vm->tree_memo || !revision)
		return false;
	m = &vm->tree_memo[gpu_m4_provider_tree_memo_slot(key)];
	if (!m->revision || !gpu_m4_provider_tree_memo_same(&m->key, key))
		goto miss;
	if (m->revision == revision) {
		gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_MEMO_HITS, 1);
		return true;
	}
	/* Proven at m->revision; later binds elsewhere leave it standing. */
	if (!gpu_m4_vm_tree_unchanged(vm->tree, m->revision, key->va, key->size)) {
		gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_MEMO_STALE, 1);
		return false;
	}
	m->revision = revision;
	gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_MEMO_LOG_HITS, 1);
	return true;
miss:
	gpu_m4_drm_provider_tree_count(GPU_M4_DRM_PROVIDER_TREE_MEMO_MISSES, 1);
	return false;
}

static void gpu_m4_provider_tree_memo_remember(struct gpu_m4_drm_provider_version *v,
	const struct gpu_m4_drm_uat_span_key *key)
{
	struct gpu_m4_drm_provider_vm *vm = v->vm;
	struct gpu_m4_provider_tree_memo *m;

	if (!READ_ONCE(tree_span_memo) || !vm->tree_memo_pending)
		return;
	if (!vm->tree_memo) {
		vm->tree_memo = kvcalloc(GPU_M4_PROVIDER_TREE_MEMO, sizeof(*vm->tree_memo),
			GFP_KERNEL);
		if (!vm->tree_memo)
			return;
	}
	m = &vm->tree_memo[gpu_m4_provider_tree_memo_slot(key)];
	m->key = *key;
	m->revision = vm->tree_memo_pending;
}

/* REMOVE-DEBUG site: the strict probe names its own (see uat.h). */
static bool gpu_m4_drm_provider_version_span_site_locked(struct gpu_m4_drm_provider_version *v,
	const struct gpu_m4_drm_uat_span_key *key, bool strict)
{
	if (IS_ERR_OR_NULL(v) || !key)
		return false;
	lockdep_assert_held(gpu_m4_drm_provider_lock(v->vm->provider));
	/* A strict probe only ever finds a strict (kind 2) proof. */
	if (gpu_m4_drm_provider_version_tree(v) && v->vm->tree &&
	    !gpu_m4_drm_provider_version_span_ready(v, key->va, key->size, key->access))
		return gpu_m4_provider_tree_memo_find(v, key);
	if (v->state != GPU_M4_PROVIDER_VERSION_GRAPH ||
	    gpu_m4_drm_provider_version_identity_locked(v->vm->provider, v->vm, v, false))
		return false;
	return strict ? gpu_m4_drm_uat_span_cached_site(v->uat, key, GPU_M4_DRM_UAT_SITE_STRICT) :
		gpu_m4_drm_uat_span_cached(v->uat, key);
}

bool gpu_m4_drm_provider_version_span_cached_locked(struct gpu_m4_drm_provider_version *v,
	const struct gpu_m4_drm_uat_span_key *key)
{
	return gpu_m4_drm_provider_version_span_site_locked(v, key, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_span_cached_locked);

void gpu_m4_drm_provider_version_span_remember_locked(struct gpu_m4_drm_provider_version *v,
	const struct gpu_m4_drm_uat_span_key *key)
{
	if (IS_ERR_OR_NULL(v) || !key)
		return;
	lockdep_assert_held(gpu_m4_drm_provider_lock(v->vm->provider));
	if (gpu_m4_drm_provider_version_tree(v) && v->vm->tree)
		gpu_m4_provider_tree_memo_remember(v, key);
	else if (v->state == GPU_M4_PROVIDER_VERSION_GRAPH)
		gpu_m4_drm_uat_span_remember(v->uat, key);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_span_remember_locked);

struct gpu_m4_drm_provider_object_span {
	struct gpu_m4_drm_provider_object *object;
	const void *identity;
	u64 va, offset;
	u32 access;
	bool exact;
};

static int gpu_m4_drm_provider_object_span_page(void *cookie, u64 page,
	const struct gpu_m4_drm_vm_leaf *leaf)
{
	const struct gpu_m4_drm_provider_object_span *c = cookie;
	u64 offset = c->offset + (page - c->va), pa;
	int ret = gpu_m4_drm_provider_object_page_locked(c->object, offset, &pa);

	if (ret)
		return ret;
	if (leaf->pa != pa || leaf->identity != c->identity || leaf->object_offset != offset ||
	    (c->exact && (leaf->access != c->access || leaf->cpu_policy != GPU_M4_DRM_CPU_NC))) {
		/* REMOVE-DEBUG: the installed leaf against the object's admitted pin. */
		pr_warn("GPU version object span: page=%#llx pa=%#llx/%#llx identity=%d offset=%#llx/%#llx access=%#x/%#x policy=%u exact=%d\n",
			(unsigned long long)page, (unsigned long long)leaf->pa,
			(unsigned long long)pa, leaf->identity == c->identity,
			(unsigned long long)leaf->object_offset, (unsigned long long)offset,
			leaf->access, c->access, (u32)leaf->cpu_policy, c->exact);
		return -ESTALE;
	}
	return 0;
}

int gpu_m4_drm_provider_version_object_locked(struct gpu_m4_drm_provider_version *v,
	u64 va, struct gpu_m4_drm_provider_object *object, const void *identity,
	u64 offset, u64 size, u32 access, bool exact)
{
	struct gpu_m4_drm_provider_object_span c = {
		.object = object, .identity = identity, .va = va, .offset = offset,
		.access = access, .exact = exact,
	};
	struct gpu_m4_drm_uat_span_key key = {
		.kind = exact ? 2 : 1, .identity = object, .aux = identity,
		.va = va, .size = size, .offset = offset, .access = access,
	};
	u64 pa;
	int ret;

	if (IS_ERR_OR_NULL(v) || IS_ERR_OR_NULL(object) || !identity || !size ||
	    !gpu_m4_drm_aligned(va) || !gpu_m4_drm_aligned(size) || !gpu_m4_drm_aligned(offset))
		return -EINVAL;
	/* The object's live session/admission gate and pin, once per span. */
	ret = gpu_m4_drm_provider_object_page_locked(object, offset, &pa);
	if (ret)
		return ret;
	/* The exact proof of this same span (the same walk and per-page checks,
	 * plus each leaf's exact access and NC policy) implies this one; it is
	 * probed first, as the native BO path usually holds it.
	 */
	if (!exact) {
		struct gpu_m4_drm_uat_span_key strict = key;

		strict.kind = 2;
		if (gpu_m4_drm_provider_version_span_site_locked(v, &strict, true))
			return 0;
	}
	if (gpu_m4_drm_provider_version_span_cached_locked(v, &key))
		return 0;
	ret = gpu_m4_drm_provider_version_walk_locked(v, va, size, access,
		gpu_m4_drm_provider_object_span_page, &c);
	if (!ret)
		gpu_m4_drm_provider_version_span_remember_locked(v, &key);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_object_locked);

int gpu_m4_drm_provider_version_capture_locked(struct gpu_m4_drm_provider_version *candidate,
	struct gpu_m4_drm_provider_version **version, struct gpu_m4_drm_vm_owner **owner)
{
	struct gpu_m4_drm_vm_owner *clone;
	struct gpu_m4_drm_provider_vm *vm;
	int ret;

	if (!version || !owner)
		return -EINVAL;
	*version = NULL;
	*owner = NULL;
	if (!candidate)
		return -EINVAL;
	vm = candidate->vm;
	lockdep_assert_held(vm->provider->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm))
		return vm->provider->error ? vm->provider->error : -ENOENT;
	/* A retired composed record is no capture source either. */
	if (candidate->state < GPU_M4_PROVIDER_VERSION_LOGICAL ||
	    candidate->state > GPU_M4_PROVIDER_VERSION_GRAPH)
		return -ESTALE;
	/* An eager build's failed detached candidate can remain registered only
	 * for rollback. It is not an independently held logical capture owner.
	 */
	if (candidate->state < GPU_M4_PROVIDER_VERSION_GRAPH &&
	    candidate->private_held != candidate->private_count)
		return -ESTALE;
	/* Another reference to an immutable GRAPH: its constructed identity, not
	 * a full walk. Binding transitions check the construction proof and live
	 * root pages; fault/diagnostic boundaries refresh fully.
	 */
	if (candidate->state == GPU_M4_PROVIDER_VERSION_GRAPH) {
		ret = gpu_m4_drm_uat_check(candidate->uat);
		if (ret)
			return ret;
	}
	ret = gpu_m4_drm_provider_version_get_locked(candidate);
	if (ret)
		return ret;
	/* This exact retained pointer, never a later active lookup, is the job's
	 * immutable authority. Snapshot-only construction does not grant roots.
	 * The version's base snapshot is built once from its retained index; each
	 * capture is then only a job-local wrapper sharing that immutable map and
	 * pin inventory, with its own publication/retirement state.
	 */
	ret = gpu_m4_drm_provider_version_snapshot(candidate, NULL);
	if (ret) {
		gpu_m4_drm_provider_version_put_locked(candidate);
		return ret;
	}
	clone = gpu_m4_drm_vm_owner_clone_with_extras(candidate->snapshot, NULL, 0);
	if (IS_ERR(clone)) {
		gpu_m4_drm_provider_version_put_locked(candidate);
		return PTR_ERR(clone);
	}
	*version = candidate;
	*owner = clone;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_capture_locked);

int gpu_m4_drm_provider_vm_capture_locked(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_version **version, struct gpu_m4_drm_vm_owner **owner)
{
	struct gpu_m4_drm_provider_version *active;
	int ret;

	if (!vm) {
		if (version)
			*version = NULL;
		if (owner)
			*owner = NULL;
		return -EINVAL;
	}
	lockdep_assert_held(vm->provider->lock);
	if (!gpu_m4_drm_provider_vm_open_locked(vm)) {
		if (version)
			*version = NULL;
		if (owner)
			*owner = NULL;
		return vm->provider->error ? vm->provider->error : -ENOENT;
	}
	active = vm->active;
	ret = gpu_m4_drm_provider_version_capture_locked(active, version, owner);
	/* This exact active version passed the complete locked capture: its
	 * snapshot (and GRAPH identity) are built and checked. Later captures
	 * of it, until the next withdrawal, may skip the mutex.
	 */
	if (!ret && *version == active && active->state >= GPU_M4_PROVIDER_VERSION_SNAPSHOT) {
		spin_lock(&vm->capture_lock);
		vm->capture = active;
		vm->capture_uat = active->state == GPU_M4_PROVIDER_VERSION_GRAPH ? active->uat : NULL;
		spin_unlock(&vm->capture_lock);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_capture_locked);

int gpu_m4_drm_provider_vm_capture(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_version **version, struct gpu_m4_drm_vm_owner **owner)
{
	struct gpu_m4_drm_provider_version *v;
	struct gpu_m4_drm_vm_owner *clone;
	struct gpu_m4_drm_uat *uat;
	int ret;

	if (!vm || !version || !owner)
		return -EINVAL;
	*version = NULL;
	*owner = NULL;
	lockdep_assert_not_held(vm->provider->lock);
	/* The published ticket is still the active version (a withdrawal under
	 * both locks precedes every change), and the active owner's reference
	 * keeps it alive for this inc_not_zero.
	 */
	spin_lock(&vm->capture_lock);
	v = vm->capture;
	uat = vm->capture_uat;
	if (v && (READ_ONCE(vm->provider->error) || !refcount_inc_not_zero(&v->refs)))
		v = NULL;
	spin_unlock(&vm->capture_lock);
	if (!v)
		return -EAGAIN;
	/* Exactly the locked capture's checks of an immutable SNAPSHOT/GRAPH
	 * version: its checked constructed identity and a job-local wrapper of
	 * its one immutable snapshot. Neither needs the provider mutex.
	 */
	ret = uat ? gpu_m4_drm_uat_check(uat) : 0;
	clone = ret ? ERR_PTR(ret) : gpu_m4_drm_vm_owner_clone_with_extras(v->snapshot, NULL, 0);
	if (IS_ERR(clone)) {
		mutex_lock(vm->provider->lock);
		gpu_m4_drm_provider_version_put_locked(v);
		mutex_unlock(vm->provider->lock);
		return PTR_ERR(clone);
	}
	*version = v;
	*owner = clone;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_capture);

bool gpu_m4_drm_provider_version_native_covered_locked(
	struct gpu_m4_drm_provider_version *candidate, const struct gpu_m4_drm_vm_owner *captured)
{
	const struct gpu_m4_drm_cache_access *need, *have;
	const struct gpu_m4_drm_resv_access *need_resv, *have_resv;
	u32 n, h, nr, hr;
	u32 mask = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;

	if (!candidate || !captured || !candidate->snapshot)
		return false;
	lockdep_assert_held(gpu_m4_drm_provider_lock(candidate->vm->provider));
	if (gpu_m4_drm_vm_owner_vm(candidate->snapshot) != gpu_m4_drm_vm_owner_vm(captured))
		return false;
	need = gpu_m4_drm_vm_owner_cache_access(candidate->snapshot, &n);
	have = gpu_m4_drm_vm_owner_cache_access(captured, &h);
	need_resv = gpu_m4_drm_vm_owner_resv_access(candidate->snapshot, &nr);
	have_resv = gpu_m4_drm_vm_owner_resv_access(captured, &hr);
	if ((n && !need) || (h && !have) || (nr && !need_resv) || (hr && !have_resv))
		return false;
	/* vm_owner_add pairs each unique native cache owner with its resv row;
	 * clone_with_extras retains that inventory as an unchanged prefix. Prove
	 * the complete prefix from the LIVE arrays, not a remembered approval.
	 * Reordered or otherwise unmatched inventories retain the general subset
	 * comparison below, including its corruption/permission rejection rules.
	 */
	if (n == nr && h >= n && hr >= nr) {
		bool prefix = true;

		for (u32 i = 0; i < n; i++) {
			const struct gpu_m4_drm_cache_access *a = &need[i], *b = &have[i];
			const struct gpu_m4_drm_resv_access *ar = &need_resv[i], *br = &have_resv[i];
			struct drm_gem_object *object;
			u64 length;

			if (IS_ERR_OR_NULL(a->pin) || !a->size || !a->access || (a->access & ~mask)) {
				prefix = false;
				break;
			}
			object = gpu_m4_drm_bo_pin_object(a->pin);
			length = gpu_m4_drm_bo_pin_size(a->pin);
			if (!object || a->offset >= length || a->size > length - a->offset ||
			    b->pin != a->pin || gpu_m4_drm_bo_pin_object(b->pin) != object ||
			    !b->size || b->offset >= length || b->size > length - b->offset ||
			    !b->access || (b->access & ~mask) || (a->access & ~b->access) ||
			    b->offset > a->offset || a->offset - b->offset >= b->size ||
			    a->size > b->size - (a->offset - b->offset) ||
			    ar->object != object || !ar->access || (ar->access & ~mask) ||
			    (a->access & ~ar->access) || br->object != object || !br->access ||
			    (br->access & ~mask) || (ar->access & ~br->access)) {
				prefix = false;
				break;
			}
		}
		if (prefix)
			return true;
	}
	for (u32 i = 0; i < n; i++) {
		const struct gpu_m4_drm_cache_access *a = &need[i];
		struct drm_gem_object *object;
		bool cache = false, reservation = false, own_reservation = false;
		u64 length;

		if (IS_ERR_OR_NULL(a->pin) || !a->size || !a->access || (a->access & ~mask))
			return false;
		object = gpu_m4_drm_bo_pin_object(a->pin);
		length = gpu_m4_drm_bo_pin_size(a->pin);
		if (!object || a->offset >= length || a->size > length - a->offset)
			return false;
		for (u32 j = 0; j < h; j++) {
			const struct gpu_m4_drm_cache_access *b = &have[j];

			if (b->pin == a->pin && gpu_m4_drm_bo_pin_object(b->pin) == object &&
			    b->size && b->offset < length && b->size <= length - b->offset &&
			    b->access && !(b->access & ~mask) && !(a->access & ~b->access) &&
			    b->offset <= a->offset && a->offset - b->offset < b->size &&
			    a->size <= b->size - (a->offset - b->offset))
				cache = true;
		}
		for (u32 j = 0; j < hr; j++)
			if (have_resv[j].object == object && have_resv[j].access &&
			    !(have_resv[j].access & ~mask) && !(a->access & ~have_resv[j].access))
				reservation = true;
		for (u32 j = 0; j < nr; j++)
			if (need_resv[j].object == object && need_resv[j].access &&
			    !(need_resv[j].access & ~mask) && !(a->access & ~need_resv[j].access))
				own_reservation = true;
		if (!cache || !reservation || !own_reservation)
			return false;
	}
	/* Resv-only growth also cannot replace a prepared cache/reservation set. */
	for (u32 i = 0; i < nr; i++) {
		bool covered = false;

		if (!need_resv[i].object || !need_resv[i].access || (need_resv[i].access & ~mask))
			return false;
		for (u32 j = 0; j < hr; j++)
			if (have_resv[j].object == need_resv[i].object && have_resv[j].access &&
			    !(have_resv[j].access & ~mask) && !(need_resv[i].access & ~have_resv[j].access))
				covered = true;
		if (!covered)
			return false;
	}
	return true;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_native_covered_locked);

u64 gpu_m4_drm_provider_version_generation(struct gpu_m4_drm_provider_version *v)
{
	return v ? v->generation : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_generation);

int gpu_m4_drm_provider_version_roots_locked(struct gpu_m4_drm_provider_version *v,
	u64 *low, u64 *high)
{
	if (!v)
		return -EINVAL;
	lockdep_assert_held(v->vm->provider->lock);
	if (gpu_m4_drm_provider_version_tree(v))
		return refcount_read(&v->refs) ? gpu_m4_drm_provider_tree_roots(v, low, high) : -ENOENT;
	if (v->state != GPU_M4_PROVIDER_VERSION_GRAPH || !v->uat)
		return -ENODATA;
	if (!refcount_read(&v->refs))
		return -ENOENT;
	/* Checked root getter of the immutable GRAPH version: state, retained
	 * root allocation and recorded descriptor, without another leaf walk.
	 */
	return gpu_m4_drm_uat_roots(v->uat, low, high);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_roots_locked);

static int gpu_m4_drm_provider_version_refresh(struct gpu_m4_drm_provider_version *v,
	bool full)
{
	struct gpu_m4_drm_provider *p;
	int ret;

	if (!v)
		return -EINVAL;
	p = v->vm->provider;
	lockdep_assert_held(p->lock);
	/* S3c: a live tree has no construction proof; its consistency is the
	 * tree's own (every edit checked, faulted otherwise).
	 */
	if (gpu_m4_drm_provider_version_tree(v))
		return gpu_m4_vm_tree_healthy(v->vm->tree);
	if (v->state != GPU_M4_PROVIDER_VERSION_GRAPH)
		return -ENODATA;
	/* Binding changes only which immutable graph is selected: an ordinary
	 * refresh consumes the construction proof and checks the live root page.
	 * A provider fault, a previously failed check/scan, or an explicit
	 * fault/diagnostic request rescans the complete manifest. A complete scan
	 * never creates a node proof: unproven nodes still fail the check.
	 */
	if (!full && v->refreshed_revision == p->fault_revision)
		ret = gpu_m4_drm_uat_refresh_check(v->uat);
	else
		ret = gpu_m4_drm_uat_refresh_verify(v->uat);
	/* A failed targeted check is never cached; the next call rescans. */
	v->refreshed_revision = ret ? 0 : p->fault_revision;
	return ret;
}

int gpu_m4_drm_provider_version_refresh_locked(struct gpu_m4_drm_provider_version *v)
{
	return gpu_m4_drm_provider_version_refresh(v, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_refresh_locked);

int gpu_m4_drm_provider_version_refresh_full_locked(struct gpu_m4_drm_provider_version *v)
{
	return gpu_m4_drm_provider_version_refresh(v, true);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_refresh_full_locked);

static bool gpu_m4_drm_provider_evidence_overlap(const void *storage, unsigned long bytes,
					       const void *owner, unsigned long owner_bytes)
{
	unsigned long start = (unsigned long)storage, base = (unsigned long)owner;

	return bytes && start < base + owner_bytes && base < start + bytes;
}

int gpu_m4_drm_provider_version_export_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_vm *vm, struct gpu_m4_drm_provider_version *version,
	struct gpu_m4_drm_provider_version_evidence *header,
	struct gpu_m4_drm_uat_table_evidence *tables, u32 capacity)
{
	struct gpu_m4_drm_provider_version_evidence result;
	struct gpu_m4_drm_provider_version *owned_version;
	bool found = false;
	unsigned long bytes = (unsigned long)capacity * sizeof(*tables);
	int ret;

	if (!p || !vm || !version || !header || (capacity && !tables) ||
	    capacity > GPU_M4_DRM_UAT_MAX_TABLES ||
	    (unsigned long)header > ~0UL - sizeof(*header) ||
	    (tables && (unsigned long)tables > ~0UL - bytes))
		return -EINVAL;
	lockdep_assert_held(p->lock);
	found = gpu_m4_drm_provider_vm_member(p, vm);
	if (!found)
		return -ENOENT;
	found = false;
	list_for_each_entry(owned_version, &vm->versions, link)
		if (owned_version == version)
			found = true;
	if (!found || !refcount_read(&version->refs))
		return -ENOENT;
	if (vm->provider != p || version->vm != vm)
		return -EPERM;
	if (version->state != GPU_M4_PROVIDER_VERSION_GRAPH || !version->uat)
		return -ENODATA;
	if (gpu_m4_drm_provider_evidence_overlap(header, sizeof(*header), p, sizeof(*p)) ||
	    gpu_m4_drm_provider_evidence_overlap(header, sizeof(*header), vm, sizeof(*vm)) ||
	    gpu_m4_drm_provider_evidence_overlap(header, sizeof(*header), version, sizeof(*version)) ||
	    gpu_m4_drm_provider_evidence_overlap(tables, bytes, p, sizeof(*p)) ||
	    gpu_m4_drm_provider_evidence_overlap(tables, bytes, vm, sizeof(*vm)) ||
	    gpu_m4_drm_provider_evidence_overlap(tables, bytes, version, sizeof(*version)) ||
	    gpu_m4_drm_provider_evidence_overlap(tables, bytes, header, sizeof(*header)))
		return -EINVAL;
	ret = gpu_m4_drm_uat_evidence_destination(version->uat, header, sizeof(*header));
	if (ret)
		return ret;
	result = (struct gpu_m4_drm_provider_version_evidence) {
		.binding_generation = p->binding[version->binding_asid ?: 1].generation,
		.provider_error = p->error,
		.flags = (vm->closed ? GPU_M4_DRM_PROVIDER_EVIDENCE_CLOSED : 0) |
			 (version->protected ? GPU_M4_DRM_PROVIDER_EVIDENCE_PROTECTED : 0) |
			 (version->working ? GPU_M4_DRM_PROVIDER_EVIDENCE_WORKING : 0),
	};
	ret = gpu_m4_drm_uat_export(version->uat, &result.uat, tables, capacity);
	if (!ret || ret == -ENOSPC)
		*header = result;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_export_locked);

int gpu_m4_drm_provider_version_page_locked(struct gpu_m4_drm_provider_version *v,
	u64 va, u32 access, struct gpu_m4_drm_vm_leaf *leaf)
{
	if (!v)
		return -EINVAL;
	lockdep_assert_held(v->vm->provider->lock);
	if (gpu_m4_drm_provider_version_tree(v))
		return gpu_m4_vm_tree_page(v->vm->tree, va, access, leaf);
	if (v->state != GPU_M4_PROVIDER_VERSION_GRAPH)
		return -ENODATA;
	return gpu_m4_drm_uat_page(v->uat, va, access, leaf);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_page_locked);

bool gpu_m4_drm_provider_version_same_image_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_version *old, struct gpu_m4_drm_provider_version *v)
{
	u32 asid = 1;

	if (IS_ERR_OR_NULL(p) || IS_ERR_OR_NULL(old) || IS_ERR_OR_NULL(v) || old == v)
		return false;
	lockdep_assert_held(p->lock);
	/* old's own binding, found by identity before any dereference of it. */
	while (asid < GPU_M4_DRM_PROVIDER_ASIDS && p->binding[asid].resident != old)
		asid++;
	if (p->error || asid == GPU_M4_DRM_PROVIDER_ASIDS || p->binding[asid].pending ||
	    !p->binding[asid].generation || old->binding_asid != asid ||
	    !old->protected || old->working || v->protected || v->working || v->binding_asid)
		return false;
	/* Registered membership and an open VM precede every dereference of v. */
	if (gpu_m4_drm_provider_version_identity_locked(p, old->vm, v, true) || v->vm != old->vm ||
	    old->state != GPU_M4_PROVIDER_VERSION_GRAPH ||
	    v->state != GPU_M4_PROVIDER_VERSION_GRAPH ||
	    IS_ERR_OR_NULL(old->uat) || IS_ERR_OR_NULL(v->uat) || old->uat == v->uat)
		return false;
	return gpu_m4_drm_uat_same_image(old->uat, v->uat);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_same_image_locked);

int gpu_m4_drm_provider_asid_bind_begin_locked(struct gpu_m4_drm_provider *p, u32 asid,
	struct gpu_m4_drm_provider_version *v, u64 expected_low, u64 expected_high)
{
	struct gpu_m4_drm_provider_binding *b;
	u64 low = 0, high = 0;
	int ret;

	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_asid_valid(asid))
		return -EINVAL;
	b = &p->binding[asid];
	/* Only this ASID's own resident must be quiescent; other ASIDs' bindings
	 * and their work are untouched by this installation.
	 */
	if (!v || v->vm->provider != p || p->error || b->pending || v->binding_asid ||
	    (b->resident && b->resident->working))
		return p->error ? p->error : -EBUSY;
	if (b->generation == U64_MAX)
		return -EOVERFLOW;
	/* Binding transition: the old/new graphs' construction proofs (every
	 * named node's identity and creator proof) plus their refreshed exact
	 * root pages. Immutable L1/L2 pages are not rescanned; a fault or failed
	 * check still forces the complete scan inside the ordinary refresh.
	 */
	if (b->resident) {
		ret = gpu_m4_drm_provider_version_refresh_locked(b->resident);
		if (!ret)
			ret = gpu_m4_drm_provider_version_roots_locked(b->resident, &low, &high);
		if (ret)
			return ret;
	}
	if (low != expected_low || high != expected_high)
		return -ESTALE;
	ret = gpu_m4_drm_provider_version_refresh_locked(v);
	/* Uncertain residency is pinned before the caller's first TTBAT write.
	 * S3c: a TREE version's removals invalidate this ASID from here on.
	 */
	if (!ret)
		ret = gpu_m4_drm_provider_version_tree(v) ?
			gpu_m4_vm_tree_install_begin(v->vm->tree, asid) :
			gpu_m4_drm_uat_begin_residency(v->uat);
	if (ret)
		return ret;
	gpu_m4_drm_provider_version_get_locked(v);
	v->protected = true;
	v->binding_asid = asid;
	b->pending = v;
	gpu_m4_drm_provider_tree_bound_locked(v);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_asid_bind_begin_locked);

int gpu_m4_drm_provider_bind_begin_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_version *v, u64 expected_low, u64 expected_high)
{
	return gpu_m4_drm_provider_asid_bind_begin_locked(p, 1, v, expected_low, expected_high);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_bind_begin_locked);

/*
 * A composed version's image maps EVERY member of the baseline it was
 * composed from, and its inventory and baseline hold every member's object
 * and claim. Retained afterwards only as a completed receipt (a history's
 * job), it would pin the backing of every unit evicted since: an old,
 * rarely displaced slot owner could keep gigabytes of evicted pools alive.
 *
 * Once its one job's work actually retired (finished) AND a later binding or
 * unbind replaced it as the resident root (both refreshed and checked by the
 * caller), no firmware, in-flight, resident, pending or protected owner can
 * use that image, and no future job can reach it: jobs bind only their own
 * versions, children compose only plain captures, captures and compositions
 * refuse a RETIRED source. Release the image itself (deferred: its callbacks
 * and pages are released outside the lock; nodes another graph names stay
 * with it) and the member-only holds: baseline, objects the source does not
 * hold, and claims beyond the source's. The source's inventory, borrowed user
 * ledger and the snapshot remain, so retained receipts still prove the job's
 * origin and cache/resv phase; a RETIRED version is never bound, rebuilt,
 * captured or composed. Best effort: without host memory for the deferred
 * owner the complete version stays, released by its final put. No member
 * object here is the last hold of a live unit: its wrapper holds its own.
 */
static void gpu_m4_drm_provider_version_release_image_locked(
	struct gpu_m4_drm_provider_version *v)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;
	struct gpu_m4_drm_provider_version *source = v->ledger_source;
	struct gpu_m4_drm_provider_graph_garbage *g;
	u32 kept = 0;

	lockdep_assert_held(p->lock);
	/* The caller's final drop (one ref) releases everything anyway. */
	if (p->error || !v->finished || !v->baseline || !source ||
	    v->state != GPU_M4_PROVIDER_VERSION_GRAPH || IS_ERR_OR_NULL(v->uat) ||
	    v->protected || v->working || v->binding_asid ||
	    v->vm->capture == v || refcount_read(&v->refs) <= 1 ||
	    v->private_held > v->base_private_count || !source->objects_indexed ||
	    source->private_lease_count > v->private_lease_count)
		return;
	g = kzalloc_obj(*g);
	if (!g) {
		p->image_failures++;
		return;
	}
	*g = (struct gpu_m4_drm_provider_graph_garbage) {
		.uat = v->uat, .bytes = sizeof(*g) + v->table_bytes,
	};
	INIT_LIST_HEAD(&g->link);
	p->bytes += sizeof(*g);
	list_add_tail(&g->link, &p->garbage_graphs);
	v->uat = NULL;
	v->table_bytes = 0;
	v->state = GPU_M4_PROVIDER_VERSION_RETIRED;
	/* Stable compaction keeps the identity order (and the sorted index). */
	for (u32 i = 0; i < v->object_count; i++) {
		struct gpu_m4_drm_provider_object *o = v->objects[i];

		if (gpu_m4_drm_provider_version_object(source, o->backing.identity) == o) {
			v->objects[kept++] = o;
			continue;
		}
		gpu_m4_drm_provider_object_put_locked(o);
		p->image_objects++;
	}
	v->object_count = v->objects_sorted = kept;
	/* Composition copied the source's claims first, then the members'. */
	for (u32 i = source->private_lease_count; i < v->private_lease_count; i++)
		gpu_m4_drm_provider_private_put_locked(v->private_leases[i]);
	v->private_lease_count = source->private_lease_count;
	/* Mapping-only members took no private_held callback reference. */
	v->private_count = v->base_private_count;
	gpu_m4_drm_provider_baseline_put_locked(v->baseline);
	v->baseline = NULL;
	p->images_released++;
}

/* Bound replacement of b's resident by its pending v, then the old image's
 * release. Exact root pages of both graphs are refreshed and checked first.
 */
static int gpu_m4_drm_provider_binding_replace_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_binding *b, struct gpu_m4_drm_provider_version *v,
	u64 observed_low, u64 observed_high)
{
	struct gpu_m4_drm_provider_version *old = b->resident;
	u64 low, high;
	int ret;

	/* Runtime has completed handoff/readback/TLBI and dropped handoff. Freshly
	 * check BOTH graphs (construction proofs and refreshed exact root pages,
	 * no leaf scan) before any old resident baseline may be released.
	 */
	ret = gpu_m4_drm_provider_version_refresh_locked(v);
	if (!ret && old)
		ret = gpu_m4_drm_provider_version_refresh_locked(old);
	if (!ret)
		ret = gpu_m4_drm_provider_version_roots_locked(v, &low, &high);
	if (!ret && (low != observed_low || high != observed_high))
		ret = -EPROTO;
	if (!ret && !gpu_m4_drm_provider_version_tree(v))
		ret = gpu_m4_drm_uat_confirm_resident(v->uat);
	/* S3c: the caller already invalidated this ASID after the store that
	 * replaced old's root: old's tree removals need no TLBI from now on.
	 */
	if (!ret && old && gpu_m4_drm_provider_version_tree(old))
		gpu_m4_vm_tree_evicted(old->vm->tree);
	else if (!ret && old)
		ret = gpu_m4_drm_uat_mark_not_resident(old->uat);
	if (ret) {
		gpu_m4_drm_provider_fault_locked(p, ret);
		return ret;
	}
	b->resident = v;
	b->pending = NULL;
	if (old) {
		old->binding_asid = 0;
		old->protected = false;
		gpu_m4_drm_provider_version_release_image_locked(old);
		gpu_m4_drm_provider_version_put_locked(old);
	}
	return 0;
}

int gpu_m4_drm_provider_asid_bind_complete_locked(struct gpu_m4_drm_provider *p, u32 asid,
	struct gpu_m4_drm_provider_version *v, u64 observed_low, u64 observed_high)
{
	struct gpu_m4_drm_provider_binding *b;
	int ret;

	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_asid_valid(asid))
		return -EINVAL;
	b = &p->binding[asid];
	if (!v || b->pending != v || p->error)
		return p->error ? p->error : -EINVAL;
	ret = gpu_m4_drm_provider_binding_replace_locked(p, b, v, observed_low, observed_high);
	/* An actually changed root: only now is the new generation exposed. */
	if (!ret)
		b->generation++;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_asid_bind_complete_locked);

int gpu_m4_drm_provider_bind_complete_locked(struct gpu_m4_drm_provider *p,
	struct gpu_m4_drm_provider_version *v, u64 observed_low, u64 observed_high)
{
	return gpu_m4_drm_provider_asid_bind_complete_locked(p, 1, v, observed_low,
		observed_high);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_bind_complete_locked);

int gpu_m4_drm_provider_asid_adopt_locked(struct gpu_m4_drm_provider *p, u32 asid,
	struct gpu_m4_drm_provider_version *v, u64 observed_low, u64 observed_high)
{
	struct gpu_m4_drm_provider_binding *b;
	struct gpu_m4_drm_provider_version *old;
	u64 low = 0, high = 0;
	int ret;

	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_asid_valid(asid))
		return -EINVAL;
	b = &p->binding[asid];
	old = b->resident;
	/* same_image admits only this ASID's quiescent resident (no pending
	 * transaction, no working consumer) and an unbound candidate of its VM.
	 */
	if (p->error || !old || !v || !gpu_m4_drm_provider_version_same_image_locked(p, old, v))
		return p->error ?: -ESTALE;
	ret = gpu_m4_drm_provider_version_refresh_locked(old);
	if (!ret)
		ret = gpu_m4_drm_provider_version_roots_locked(old, &low, &high);
	if (ret)
		return ret;
	if (low != observed_low || high != observed_high)
		return -ESTALE;
	ret = gpu_m4_drm_provider_version_refresh_locked(v);
	if (!ret)
		ret = gpu_m4_drm_uat_begin_residency(v->uat);
	if (ret)
		return ret;
	/* From here the owner transfer is the bound replacement (fail-stop on
	 * any uncertainty). The installed root and its translations did not
	 * change, so the binding generation is retained.
	 */
	gpu_m4_drm_provider_version_get_locked(v);
	v->protected = true;
	v->binding_asid = asid;
	b->pending = v;
	ret = gpu_m4_drm_provider_binding_replace_locked(p, b, v, observed_low, observed_high);
	if (!ret)
		gpu_m4_drm_provider_tree_bound_locked(v);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_asid_adopt_locked);

int gpu_m4_drm_provider_asid_unbind_complete_locked(struct gpu_m4_drm_provider *p, u32 asid,
	u64 observed_low, u64 observed_high)
{
	struct gpu_m4_drm_provider_binding *b;
	struct gpu_m4_drm_provider_version *old;
	int ret;

	lockdep_assert_held(p->lock);
	if (!gpu_m4_drm_provider_asid_valid(asid))
		return -EINVAL;
	b = &p->binding[asid];
	old = b->resident;
	if (p->error || b->pending || !old || old->working)
		return p->error ? p->error : -EBUSY;
	if (observed_low || observed_high) {
		gpu_m4_drm_provider_fault_locked(p, -EPROTO);
		return -EPROTO;
	}
	ret = 0;
	if (gpu_m4_drm_provider_version_tree(old))
		gpu_m4_vm_tree_evicted(old->vm->tree);
	else
		ret = gpu_m4_drm_uat_mark_not_resident(old->uat);
	if (ret) {
		gpu_m4_drm_provider_fault_locked(p, ret);
		return ret;
	}
	/* The generation persists: a later installation in this ASID advances it. */
	b->resident = NULL;
	old->binding_asid = 0;
	old->protected = false;
	gpu_m4_drm_provider_version_release_image_locked(old);
	gpu_m4_drm_provider_version_put_locked(old);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_asid_unbind_complete_locked);

int gpu_m4_drm_provider_unbind_complete_locked(struct gpu_m4_drm_provider *p,
	u64 observed_low, u64 observed_high)
{
	return gpu_m4_drm_provider_asid_unbind_complete_locked(p, 1, observed_low,
		observed_high);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_unbind_complete_locked);

int gpu_m4_drm_provider_version_begin_work_locked(struct gpu_m4_drm_provider_version *v)
{
	struct gpu_m4_drm_provider *p = v->vm->provider;
	u32 asid = v->binding_asid;
	int ret;

	lockdep_assert_held(p->lock);
	if (p->error || !gpu_m4_drm_provider_asid_valid(asid) || p->binding[asid].pending ||
	    p->binding[asid].resident != v ||
	    (v->working && !gpu_m4_drm_provider_version_tree(v)))
		return p->error ? p->error : -EBUSY;
	/* S3c: one TREE version serves every job of the VM: count them. */
	if (gpu_m4_drm_provider_version_tree(v)) {
		if (v->tree_work == U32_MAX)
			return -EOVERFLOW;
		v->tree_work++;
		v->working = true;
		return 0;
	}
	ret = gpu_m4_drm_uat_begin_work(v->uat);
	if (!ret)
		v->working = true;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_begin_work_locked);

int gpu_m4_drm_provider_version_retire_locked(struct gpu_m4_drm_provider_version *v)
{
	int ret;

	lockdep_assert_held(v->vm->provider->lock);
	if (!v->working || v->vm->provider->error)
		return v->vm->provider->error ? v->vm->provider->error : -EINVAL;
	/* S3c: the end of one job's work on the TREE version (O(1)). */
	if (gpu_m4_drm_provider_version_tree(v)) {
		if (WARN_ON(!v->tree_work))
			return -EPROTO;
		v->working = --v->tree_work != 0;
		return 0;
	}
	ret = gpu_m4_drm_uat_retire(v->uat);
	if (!ret)
		v->working = false;
	/* Still the resident root: its image is released only after a later
	 * binding or unbind replaced it (version_release_image).
	 */
	if (!ret && v->baseline)
		v->finished = true;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_version_retire_locked);

void gpu_m4_drm_provider_vm_close_locked(struct gpu_m4_drm_provider_vm *vm)
{
	lockdep_assert_held(vm->provider->lock);
	if (vm->closed)
		return;
	gpu_m4_drm_provider_vm_withdraw_locked(vm);
	WRITE_ONCE(vm->closed, true);
	if (vm->authority) {
		/* The installed tree outlives close (residency, guards): its
		 * final free destroys it.
		 */
		gpu_m4_drm_provider_tree_authority_close_locked(vm);
	} else {
		gpu_m4_vm_tree_destroy(vm->tree);
		vm->tree = NULL;
	}
	gpu_m4_drm_provider_graph_cache_drop_locked(vm);
	/* Membership becomes undiscoverable; composed versions keep theirs. */
	gpu_m4_drm_provider_baseline_put_locked(vm->baseline);
	vm->baseline = NULL;
	if (vm->active) {
		gpu_m4_drm_provider_version_put_locked(vm->active);
		vm->active = NULL;
	}
	gpu_m4_drm_provider_vm_put_locked(vm);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_close_locked);

void gpu_m4_drm_provider_vm_close(struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider *p = vm->provider;

	mutex_lock(p->lock);
	gpu_m4_drm_provider_vm_close_locked(vm);
	mutex_unlock(p->lock);
	gpu_m4_drm_provider_reap(p);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_close);

int gpu_m4_drm_provider_vm_destroy_locked(struct gpu_m4_drm_provider_vm *vm)
{
	const struct gpu_m4_drm_provider_baseline *b = vm->baseline;
	struct gpu_m4_drm_provider_object *o;
	u32 objects = 0, baseline_leases = 0;
	int ret = 0;

	lockdep_assert_held(vm->provider->lock);
	/* S3c: no version captures this VM's mappings. Destroy is its close:
	 * guarded user extents wait for the last guard, the tree for its last
	 * TREE version reference (jobs, residency) at the final free.
	 */
	if (vm->authority) {
		if (vm->closed)
			return -ENOENT;
		gpu_m4_drm_provider_vm_close_locked(vm);
		return 0;
	}
	/* Active-only native pins and private claims are destroyable, as are the
	 * idle native baseline's own membership holds (dropped by the close).
	 * Any queue, external claim ref, old version, job or residency adds a
	 * retained owner.
	 */
	list_for_each_entry(o, &vm->objects, link) {
		u32 tree = gpu_m4_vm_tree_holds(vm->tree, o); /* shadow tree */
		u32 expected = 1 + tree;

		objects++;
		for (u32 i = 0; b && i < b->object_count; i++)
			if (b->objects[i] == o && vm->active &&
			    gpu_m4_drm_provider_version_object(vm->active, o->backing.identity))
				expected = 2 + tree;
		if (refcount_read(&o->refs) != expected)
			ret = -EBUSY;
	}
	for (u32 i = 0; b && i < b->lease_count; i++) {
		bool active = false;

		for (u32 j = 0; vm->active && j < vm->active->private_lease_count; j++)
			active |= vm->active->private_leases[j] == b->leases[i];
		if (!active) {
			baseline_leases++;
			if (refcount_read(&b->leases[i]->refs) != 1)
				ret = -EBUSY;
		}
	}
	if (vm->active) {
		if (vm->private_lease_count != vm->active->private_lease_count + baseline_leases)
			ret = -EBUSY;
		for (u32 i = 0; i < vm->active->private_lease_count; i++) {
			u32 expected = 1;

			for (u32 j = 0; b && j < b->lease_count; j++)
				expected += b->leases[j] == vm->active->private_leases[i];
			if (refcount_read(&vm->active->private_leases[i]->refs) != expected)
				ret = -EBUSY;
		}
	}
	if (vm->closed)
		ret = -ENOENT;
	else if (refcount_read(&vm->refs) != 2 + objects + vm->private_lease_count + !!b ||
		 !vm->active ||
		 refcount_read(&vm->active->refs) != 1 || vm->active->protected ||
		 vm->active->working)
		ret = -EBUSY;
	else if (!ret)
		gpu_m4_drm_provider_vm_close_locked(vm);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_destroy_locked);

int gpu_m4_drm_provider_vm_destroy(struct gpu_m4_drm_provider_vm *vm)
{
	struct gpu_m4_drm_provider *p = vm->provider;
	int ret;

	mutex_lock(p->lock);
	ret = gpu_m4_drm_provider_vm_destroy_locked(vm);
	mutex_unlock(p->lock);
	if (!ret)
		gpu_m4_drm_provider_reap(p);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_vm_destroy);

void gpu_m4_drm_provider_reap(struct gpu_m4_drm_provider *p)
{
	struct gpu_m4_drm_provider_vm *each;

	if (!p || gpu_m4_drm_provider_get(p))
		return;
	lockdep_assert_not_held(p->lock);
	/* Reclaim before the refused request retries: every VM's construction
	 * cache is optional and idle (a borrowed one is refunded after its last
	 * borrower), so pressure hands it to the garbage drained below. Caching
	 * resumes with the next materialization once no charge is refused.
	 */
	mutex_lock(p->lock);
	if (p->pressure) {
		list_for_each_entry(each, &p->vms, link)
			gpu_m4_drm_provider_graph_cache_drop_locked(each);
		p->pressure = false;
	}
	mutex_unlock(p->lock);
	/* Caller holds a provider reference. Objects stay charged/VM-reachable
	 * while removed from the garbage queue and freed outside the lock.
	 */
	for (;;) {
		struct gpu_m4_drm_provider_version *v;
		struct gpu_m4_drm_provider_object *o;
		struct gpu_m4_drm_provider_vm *vm;
		int ret;

		mutex_lock(p->lock);
		if (!list_empty(&p->garbage_graphs)) {
			struct gpu_m4_drm_provider_graph_garbage *g;

			g = list_first_entry(&p->garbage_graphs,
				struct gpu_m4_drm_provider_graph_garbage, link);
			list_del_init(&g->link);
			mutex_unlock(p->lock);
			ret = gpu_m4_drm_uat_release(g->uat);
			mutex_lock(p->lock);
			if (ret) {
				/* A failed halt/release cannot become a dropped charge or a
				 * replacement graph. Retain this exact version and artifact.
				 */
				if (g->version)
					g->version->protected = true;
				list_add_tail(&g->link, &p->retained_graphs);
				gpu_m4_drm_provider_fault_locked(p, ret);
				mutex_unlock(p->lock);
				continue;
			}
			p->bytes -= g->bytes;
			/* A construction-cache image has no version owner. */
			if (g->version)
				gpu_m4_drm_provider_version_put_locked(g->version);
			/* The partial graph no longer names any shared base page:
			 * only now return its borrow on their charge owner.
			 */
			{
				struct gpu_m4_drm_provider_graph_borrow borrow = {
					.version = g->base_version, .cache = g->base_cache,
				};

				gpu_m4_drm_provider_borrow_put_locked(p, &borrow);
			}
			mutex_unlock(p->lock);
			kfree(g);
			continue;
		}
		if (!list_empty(&p->garbage_versions)) {
			u64 charge;

			v = list_first_entry(&p->garbage_versions,
				struct gpu_m4_drm_provider_version, garbage);
			list_del_init(&v->garbage);
			mutex_unlock(p->lock);
			ret = v->uat ? gpu_m4_drm_uat_release(v->uat) : 0;
			if (!ret)
				v->uat = NULL;
			if (!ret && v->snapshot)
				ret = gpu_m4_drm_vm_owner_release(v->snapshot);
			if (!ret) {
				v->snapshot = NULL;
				ret = gpu_m4_drm_vm_owner_release(v->lineage);
			}
			if (!ret)
				for (u32 i = 0; i < v->private_held; i++)
					v->private[i].put(v->private[i].cookie);
			mutex_lock(p->lock);
			if (ret) {
				/* Unexpected protected final drop never loses its charge. */
				refcount_set(&v->refs, 1);
				v->protected = true;
				gpu_m4_drm_provider_fault_locked(p, ret);
				mutex_unlock(p->lock);
				continue;
			}
			charge = v->table_bytes + v->metadata_bytes;
			vm = v->vm;
			gpu_m4_drm_provider_version_unlink(v);
			for (u32 i = 0; i < v->object_count; i++)
				gpu_m4_drm_provider_object_put_locked(v->objects[i]);
			for (u32 i = 0; i < v->private_lease_count; i++)
				gpu_m4_drm_provider_private_put_locked(v->private_leases[i]);
			gpu_m4_drm_provider_baseline_put_locked(v->baseline);
			v->baseline = NULL;
			/* A borrowed ledger stays its source's (reaped after v). */
			if (v->ledger_source) {
				gpu_m4_drm_provider_version_put_locked(v->ledger_source);
				v->ledger.leaf = NULL;
				v->index = (struct gpu_m4_drm_vm_ledger_index) { 0 };
			}
			mutex_unlock(p->lock);
			kvfree(v->ledger.leaf);
			gpu_m4_drm_vm_ledger_index_free(&v->index);
			gpu_m4_drm_vm_ledger_index_free(&v->manifest_index);
			kvfree(v->manifest);
			kfree(v->objects);
			kfree(v->object_used);
			kvfree(v->private);
			kvfree(v->native_private);
			kfree(v->private_leases);
			kfree(v);
			mutex_lock(p->lock);
			p->bytes -= charge;
			gpu_m4_drm_provider_vm_put_locked(vm);
			mutex_unlock(p->lock);
			continue;
		}
		if (!list_empty(&p->garbage_objects)) {
			u64 charge;

			o = list_first_entry(&p->garbage_objects,
				struct gpu_m4_drm_provider_object, garbage);
			list_del_init(&o->garbage);
			mutex_unlock(p->lock);
			gpu_m4_drm_bo_pin_put(o->pin);
			kvfree(o->backing.pa);
			charge = o->charge;
			vm = o->vm;
			kfree(o);
			mutex_lock(p->lock);
			p->bytes -= charge;
			gpu_m4_drm_provider_vm_put_locked(vm);
			mutex_unlock(p->lock);
			continue;
		}
		if (!list_empty(&p->garbage_private)) {
			struct gpu_m4_drm_provider_private_lease *lease;

			lease = list_first_entry(&p->garbage_private,
				struct gpu_m4_drm_provider_private_lease, garbage);
			list_del_init(&lease->garbage);
			vm = lease->vm;
			mutex_unlock(p->lock);
			kfree(lease);
			mutex_lock(p->lock);
			p->bytes -= sizeof(*lease);
			gpu_m4_drm_provider_vm_put_locked(vm);
			mutex_unlock(p->lock);
			continue;
		}
		if (!list_empty(&p->garbage_vms)) {
			vm = list_first_entry(&p->garbage_vms,
				struct gpu_m4_drm_provider_vm, garbage);
			list_del_init(&vm->garbage);
			gpu_m4_drm_provider_vm_unlist_locked(p, vm);
			mutex_unlock(p->lock);
			/* S3c: tree (evicted, unguarded), its holds, then the context
			 * its DRIVER extent named. A refused tree destruction retains
			 * the complete VM owner (fail-stop, like a refused graph).
			 */
			ret = gpu_m4_drm_provider_tree_authority_free(vm);
			if (ret) {
				mutex_lock(p->lock);
				refcount_set(&vm->refs, 1);
				list_add_tail(&vm->garbage, &p->retained_vms);
				gpu_m4_drm_provider_fault_locked(p, ret);
				mutex_unlock(p->lock);
				continue;
			}
			gpu_m4_drm_provider_context_free(&vm->context);
			gpu_m4_drm_vm_put(vm->frontend);
			kvfree(vm->tree_memo);
			kfree(vm);
			mutex_lock(p->lock);
			p->bytes -= GPU_M4_DRM_UAT_CONTEXT_SIZE + sizeof(*vm);
			mutex_unlock(p->lock);
			gpu_m4_drm_provider_put(p);
			continue;
		}
		mutex_unlock(p->lock);
		gpu_m4_drm_provider_put(p);
		return;
	}
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_reap);

MODULE_DESCRIPTION("Private Apple M4 retained provider VM lifecycle");
#ifdef CONFIG_DEBUG_FS
static struct dentry *gpu_m4_drm_provider_vm_debugfs;

static int gpu_m4_drm_provider_vm_stats_show(struct seq_file *m, void *unused)
{
	char *buffer = kzalloc(PAGE_SIZE, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;
	gpu_m4_drm_provider_tree_stats_get(buffer, NULL);
	seq_puts(m, buffer);
	kfree(buffer);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gpu_m4_drm_provider_vm_stats);

static int __init gpu_m4_drm_provider_vm_debugfs_init(void)
{
	gpu_m4_drm_provider_vm_debugfs = debugfs_create_dir(KBUILD_MODNAME, NULL);
	debugfs_create_file("authority_stats", 0400, gpu_m4_drm_provider_vm_debugfs, NULL, &gpu_m4_drm_provider_vm_stats_fops);
	return 0;
}

static void __exit gpu_m4_drm_provider_vm_debugfs_exit(void)
{
	debugfs_remove_recursive(gpu_m4_drm_provider_vm_debugfs);
}
module_init(gpu_m4_drm_provider_vm_debugfs_init);
module_exit(gpu_m4_drm_provider_vm_debugfs_exit);
#endif

MODULE_LICENSE("GPL");
