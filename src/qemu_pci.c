#include "qemu_pci.h"
#include "platform.h"
#include "mmio.h"
#include "uart.h"

#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT

#define PCI_VENDOR_INVALID 0xFFFFU
#define PCI_COMMAND        0x04U
#define PCI_CLASS_REV      0x08U
#define PCI_BAR0           0x10U

static inline u64 pci_cfg_addr(u32 bus, u32 dev, u32 fn, u32 reg)
{
    return QEMU_PCI_ECAM_BASE +
           ((u64)bus << 20) + ((u64)dev << 15) +
           ((u64)fn << 12) + (u64)reg;
}

static u32 pci_cfg_read(u32 bus, u32 dev, u32 fn, u32 reg)
{
    return mmio_read(pci_cfg_addr(bus, dev, fn, reg));
}

static void pci_cfg_write(u32 bus, u32 dev, u32 fn, u32 reg, u32 value)
{
    mmio_write(pci_cfg_addr(bus, dev, fn, reg), value);
}

bool qemu_pci_find_class(u32 wanted_class, struct pci_device *out)
{
    if (!out)
        return false;

    for (u32 dev = 0; dev < 32U; dev++) {
        u32 vendor = pci_cfg_read(0, dev, 0, 0) & 0xFFFFU;
        if (vendor == PCI_VENDOR_INVALID)
            continue;
        for (u32 fn = 0; fn < 8U; fn++) {
            u32 id = pci_cfg_read(0, dev, fn, 0);
            if ((id & 0xFFFFU) == PCI_VENDOR_INVALID)
                continue;
            u32 class_code = (pci_cfg_read(0, dev, fn, PCI_CLASS_REV) >> 8) & 0xFFFFFFU;
            if (class_code != wanted_class)
                continue;

            u32 bar = pci_cfg_read(0, dev, fn, PCI_BAR0);
            if ((bar & 1U) != 0U || (bar & 0xFFFFFFF0U) == 0U)
                bar = 0U;
            if (bar == 0U) {
                /* QEMU leaves optional PCI BARs unassigned without firmware.
                 * Allocate the first aligned slot in the virt MMIO aperture. */
                pci_cfg_write(0, dev, fn, PCI_BAR0, 0x10000000U);
                bar = pci_cfg_read(0, dev, fn, PCI_BAR0);
            }
            if ((bar & 1U) != 0U || (bar & 0xFFFFFFF0U) == 0U)
                return false;
            u64 base = (u64)(bar & 0xFFFFFFF0U);
            if ((bar & 6U) == 4U) {
                u32 high = pci_cfg_read(0, dev, fn, PCI_BAR0 + 4U);
                base |= (u64)high << 32;
            }
            if (base == 0U || base > 0xFFFFFFFFULL)
                return false;

            u32 command = pci_cfg_read(0, dev, fn, PCI_COMMAND);
            pci_cfg_write(0, dev, fn, PCI_COMMAND, command | 0x0006U);
            uart_puts("[pci] QEMU xHCI dev=");
            uart_hex(dev);
            uart_puts(" BAR0=");
            uart_hex(bar);
            uart_puts(" base=");
            uart_hex((u32)base);
            uart_puts(" CMD=");
            uart_hex(pci_cfg_read(0, dev, fn, PCI_COMMAND));
            uart_puts("\n");
            out->bus = 0;
            out->dev = dev;
            out->func = fn;
            out->vendor_id = (u16)(id & 0xFFFFU);
            out->device_id = (u16)(id >> 16);
            out->class_code = class_code;
            out->bar0 = base;
            out->bar0_64 = (bar & 6U) == 4U;
            return true;
        }
    }
    return false;
}

#else

bool qemu_pci_find_class(u32 class_code, struct pci_device *out)
{
    (void)class_code;
    (void)out;
    return false;
}

#endif
