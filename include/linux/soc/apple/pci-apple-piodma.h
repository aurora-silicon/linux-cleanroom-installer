/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_APPLE_PCI_PIODMA_H
#define _LINUX_SOC_APPLE_PCI_PIODMA_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct pci_dev;

#if IS_ENABLED(CONFIG_PCIE_APPLE_PIODMA_DIAG)
/* Require completed Neo admission and permanent retained-DMA protection. */
int apple_piodma_radio_check(struct pci_dev *pdev);
/* J700 radio function 0 on a bound, bootloader-retained (non-PIODMA) host. */
bool apple_pcie_radio_retained(struct pci_dev *pdev);
#else
static inline int apple_piodma_radio_check(struct pci_dev *pdev)
{
	return -ENODEV;
}

static inline bool apple_pcie_radio_retained(struct pci_dev *pdev)
{
	return false;
}
#endif

#endif /* _LINUX_SOC_APPLE_PCI_PIODMA_H */
