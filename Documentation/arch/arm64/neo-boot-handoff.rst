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

The retained MTP session also needs its SID0 identity mapping kept intact.
The adapter admits only J700's explicit retained-bypass property at the MTP
DART, validates bypass and an empty translation root, keeps the provider
powered, and selects identity DMA for its clients without rewriting the APF
policy. Multi-touch AFE reset stays asserted during firmware upload; the
retained session then receives the power-on handshake without a second off
request after upload. Other machines and firmware-managed reset keep their
existing sequences.

The qualified older radio handoff has firmware-initialized PCIe without a
PIODMA supplier. On J700 only, retain its RID/SID mappings, require an already
up link, and expose bus 1 function 0. Function 1 remains firmware-owned and
hidden. Adopt an existing RID mapping only if it exactly matches the requested
SID and RID. The DART adapter admits only the radio address, 19-stream geometry,
Linux-owned SID1 and firmware SID16/18 mask; it does not reset unrelated streams
or change their APF policy. Newer PIODMA experiments remain gated as before.

These local adapters are implemented in the CLEANROOM source lane from the
observed retained loader contract, with comparison against the working
Dirtyroom integration. They do not certify independent cleanroom provenance.

The inherited MTP reset GPIO uses the SMC gP13 pin API, rather than the pcIO
function API used by newer cold-start profiles. Select that existing generic
GPIO implementation only for an explicitly inherited J700 MTP helper; radio
power still uses its pcIO command. Build 3 reached the greeter and joined Wi-Fi,
but the inappropriate pcIO AFE reset left multi-touch waiting for readiness.
