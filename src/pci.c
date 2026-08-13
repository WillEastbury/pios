#include "pci.h"
#include "platform.h"
#include "pcie.h"
#include "qemu_pci.h"

#define PCI_CLASS_XHCI 0x0C0330U
#define PCI_COMMAND    0x04U
#define PCI_BAR0       0x10U

bool pci_find_class(u32 class_code, struct pci_device *out)
{
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    return qemu_pci_find_class(class_code, out);
#else
    if (!out || !pcie_link_up())
        return false;
    for (u32 bus = 0; bus < 2U; bus++)
        for (u32 dev = 0; dev < 32U; dev++)
            for (u32 fn = 0; fn < 8U; fn++) {
                u32 id = pcie_cfg_read(bus, dev, fn, 0);
                if ((id & 0xFFFFU) == 0xFFFFU)
                    continue;
                u32 cc = (pcie_cfg_read(bus, dev, fn, 8) >> 8) & 0xFFFFFFU;
                if (cc != class_code)
                    continue;
                u32 bar = pcie_cfg_read(bus, dev, fn, PCI_BAR0);
                if ((bar & 1U) != 0U)
                    return false;
                out->bus = bus;
                out->dev = dev;
                out->func = fn;
                out->vendor_id = (u16)(id & 0xFFFFU);
                out->device_id = (u16)(id >> 16);
                out->class_code = cc;
                out->bar0 = (u64)(bar & 0xFFFFFFF0U);
                out->bar0_64 = (bar & 6U) == 4U;
                return true;
            }
    return false;
#endif
}

bool pci_enable_device(const struct pci_device *dev)
{
    if (!dev)
        return false;
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    return true; /* qemu_pci_find_class enables the QEMU device. */
#else
    u32 cmd = pcie_cfg_read(dev->bus, dev->dev, dev->func, PCI_COMMAND);
    pcie_cfg_write(dev->bus, dev->dev, dev->func, PCI_COMMAND, cmd | 0x6U);
    return true;
#endif
}
