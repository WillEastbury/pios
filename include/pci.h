#pragma once

#include "types.h"

struct pci_device {
    u32 bus;
    u32 dev;
    u32 func;
    u16 vendor_id;
    u16 device_id;
    u32 class_code;
    u64 bar0;
    bool bar0_64;
};

bool pci_find_class(u32 class_code, struct pci_device *out);
bool pci_enable_device(const struct pci_device *dev);
