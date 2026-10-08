.. SPDX-License-Identifier: GPL-2.0-only

MacBook Neo retained bootloader compatibility
============================================

The Semi-dirtyroom kernel can use the qualified Neo m1n1/U-Boot bundle without
replacing the boot chain shared by a working Dirtyroom fallback.

That bundle describes storage as ``apple,t8140-nvme-ans3`` with
``apple,t8132-nvme-ans2`` as a fallback compatible. ANS is stopped on entry to
Linux. This is a legacy boot contract: use the Neo hardware descriptor and its
FUA quirk, reject an inherited live-session property, and take the ordinary
reset/reinitialization path when CPU_CONTROL.RUN is clear. Never reset or
renegotiate an unexpectedly running ANS under this contract. The newer
``apple,t8140-nvme-ans2`` live-session contract remains unchanged.

The loader also starts the MTP helper and publishes the u32 property
``linux-enablement-mac,rtkit-inherited``. A value of one transfers its running
firmware and loader-reserved shared buffers; the helper driver checks RUN and
keeps the session intact instead of sending a second wake negotiation. A zero
or absent property follows the existing boot/wake path. Malformed values and a
stopped inherited helper fail. Removal must not quiesce a retained session for
which Linux never created an RTKit endpoint owner.

These adapters address the observed SSD boot timeout and MTP wake timeout.
They do not qualify DCP, GPU, camera, suspend or NVRAM on the combined kernel.
