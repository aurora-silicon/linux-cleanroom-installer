/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * External displays (DCPEXT0/1) are owned by the built-in m4-dcpext driver
 * (drivers/gpu/drm/apple/m4-dcpext/), which publishes its own DRM device.
 * This DCP module drives the internal panel only.
 */
struct dcp_m4_ext {
	int unused;
};

static inline void dcp_m4_ext_create_all(struct dcp_m4_ready *dcp)
{
}

static inline void dcp_m4_ext_shutdown_all(struct dcp_m4_ready *dcp)
{
}

static int dcp_m4_ext_output_error(struct dcp_m4_output *o)
{
	return 0;
}

static int dcp_m4_ext_kms_init(struct dcp_m4_kms *k, struct dcp_m4_ready *dcp)
{
	return 0;
}
