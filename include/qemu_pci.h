#pragma once

#include "types.h"
#include "pci.h"

/* QEMU virt PCIe host bridge. */
#define QEMU_PCI_ECAM_BASE 0x4010000000ULL

bool qemu_pci_find_class(u32 class_code, struct pci_device *out);
