# SPDX-License-Identifier: GPL-2.0-only OR MIT

CFLAGS_trace.o = -I$(src)

appledrm-y := apple_drv.o
appledrm-y += afk.o dcp.o dcp_backlight.o dptxep.o iomfb.o parser.o systemep.o
appledrm-$(CONFIG_DRM_APPLE_AUDIO) += audio.o
appledrm-$(CONFIG_DRM_APPLE_AUDIO) += av.o
appledrm-y += connector.o
appledrm-y += ibootep.o
appledrm-y += iomfb_v12_3.o
appledrm-y += iomfb_v13_3.o
appledrm-y += iomfb_h17p.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += iomfb_h17p_test.o
appledrm-y += epic/dpavservep.o
appledrm-y += plane.o


appledrm-$(CONFIG_TRACING) += trace.o

obj-$(CONFIG_DRM_APPLE) += appledrm.o
obj-$(CONFIG_DRM_APPLE_M4_GPU_POWER) += apple-gpu-m4-power.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-scheduler.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-memory.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-resv.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-vm-owner.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-uat.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-vm-tree.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-provider-vm.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-provider-job.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-provider-lane.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-provider-upper.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-provider-fixture.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-provider-timestamp.o
obj-$(CONFIG_DRM_APPLE_M4_GPU) += gpu-m4-drm-provider-timestamp-alias.o
obj-$(CONFIG_DRM_APPLE_KUNIT_TEST) += afk_test.o
obj-$(CONFIG_DRM_APPLE_KUNIT_TEST) += dcp_backlight_test.o
