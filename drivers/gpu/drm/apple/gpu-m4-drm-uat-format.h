/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_UAT_FORMAT_H
#define APPLE_GPU_M4_DRM_UAT_FORMAT_H

#include "gpu-m4-drm-vm-ledger.h"

/*
 * Client lower page-table format, shared by the immutable version builder
 * (gpu-m4-drm-uat.c) and the persistent per-VM tree (gpu-m4-drm-vm-tree.c).
 * 16 KiB pages and tables over a 42-bit VA: a root of 64 used entries
 * (VA[41:36]), level-1 tables of 2048 entries (VA[35:25]) and level-2 leaf
 * tables of 2048 entries (VA[24:14]). A table descriptor is its PA | 3; a leaf
 * is the page PA | gpu_m4_drm_vm_leaf_attrs(access).
 */
#define GPU_M4_UAT_ENTRIES 2048U
#define GPU_M4_UAT_ROOT_ENTRIES 64U
#define GPU_M4_UAT_VA_LIMIT (1ULL << 42)

/* Lowest VA bit the table at @level indexes: root 36, level 1 25, leaf 14. */
static inline u32 gpu_m4_uat_fmt_shift(u8 level)
{
	return level == 0 ? 36 : level == 1 ? 25 : 14;
}

/* Entry index of @va in its table at @level (the root only uses 0..63). */
static inline u32 gpu_m4_uat_fmt_slot(u64 va, u8 level)
{
	return (va >> gpu_m4_uat_fmt_shift(level)) & (GPU_M4_UAT_ENTRIES - 1);
}

/* Base VA of the table at @level covering @va: 64 GiB (level 1) or 32 MiB
 * (level 2) aligned; the root covers everything from zero.
 */
static inline u64 gpu_m4_uat_fmt_base(u64 va, u8 level)
{
	return level ? va & ~((1ULL << gpu_m4_uat_fmt_shift(level - 1)) - 1) : 0;
}

static inline u64 gpu_m4_uat_fmt_table_pte(u64 pa)
{
	return pa | 3;
}

/* Valid only for an access gpu_m4_drm_vm_leaf_attrs() accepts. */
static inline u64 gpu_m4_uat_fmt_leaf_pte(u64 pa, u32 access)
{
	return pa | gpu_m4_drm_vm_leaf_attrs(access);
}

/*
 * TLBI operands for the VM's user ASID s (CPU instructions, never TTBAT words).
 * A removed run of one 16 KiB page is VAE1OS (s << 48) | ((VA >> 12) &
 * 0xffffffffffc); a longer run is RVAE1OS (s << 48) | (TG 2 << 46) |
 * (SCALE << 44) | (NUM << 39) | ((VA >> 14) & 0x1fffffffff), TTL 0, covering
 * (NUM + 1) << (5 * SCALE + 1) pages from VA. ASIDE1OS s << 48 is only for a
 * whole-ASID ownership transition, never a page-count fallback.
 */
#define GPU_M4_UAT_TLBI_RANGE_MAX_PAGES (32ULL << 16)

static inline u64 gpu_m4_uat_fmt_tlbi_asid(u32 asid)
{
	return (u64)asid << 48;
}

static inline u64 gpu_m4_uat_fmt_tlbi_va(u32 asid, u64 va)
{
	return ((u64)asid << 48) | ((va >> 12) & 0xffffffffffcULL);
}

static inline u64 gpu_m4_uat_fmt_tlbi_range(u32 asid, u64 va, u32 scale, u32 num)
{
	return ((u64)asid << 48) | (2ULL << 46) | ((u64)(scale & 3) << 44) |
		((u64)(num & 31) << 39) | ((va >> 14) & 0x1fffffffffULL);
}

static inline u64 gpu_m4_uat_fmt_tlbi_range_pages(u32 scale, u32 num)
{
	return (u64)(num + 1) << (5 * scale + 1);
}

#endif
