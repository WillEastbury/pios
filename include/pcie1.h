/*
 * pcie1.h - BCM2712 PCIe1 (FFC / HAT) root complex
 *
 * Separate from pcie2/RP1 (src/pcie.c). CPU window, reset id, MSI and
 * inbound DMA are all distinct. Do not reuse PCIE_RC_BASE or the RP1
 * 8 MiB ATU at 0x1F00000000.
 *
 * Milestone A (#137): train the FFC link and enumerate config space.
 * Milestone B (B50 / LevelZero) is fail-closed until A is proven live.
 *
 * Linux: pcie1@1000110000 / pciex1, GIC_SPI 223/224 (INTID 255/256),
 * max-link-speed 2, num-lanes 1. Firmware needs dtparam=pciex1.
 */
#pragma once
#include "types.h"

#define PCIE1_VID_INTEL     0x8086U
#define PCIE1_DID_B50       0xE212U   /* Arc Pro B50 / Battlemage BMG-G21 */

#define PCIE1_SCAN_BUS_LO   1U
#define PCIE1_SCAN_BUS_HI   8U        /* riser switch + a few hops */
#define PCIE1_SCAN_DEV_HI   31U       /* full type-0/1 bus */
#define PCIE1_SCAN_MAX      16U
#define PCIE1_DASH_MAX      4U        /* extra HARDWARE rows for enum */

#define PCIE1_LINK_CAP_ID   0x10U     /* PCI Express capability */

struct pcie1_ep {
    u8 bus;
    u8 dev;
    u8 func;
    u16 vendor;
    u16 device;
    u8 revision;
    u8 prog_if;
    u8 subclass;
    u8 base_class;
    u8 hdr_type;       /* bit7 = multifunction */
    u8 sec_bus;        /* type-1 bridge only */
    u8 sub_bus;
};

struct pcie1_status {
    bool present;          /* PIOS_HAS_PCIE1 compile-time */
    bool inited;
    bool link_up;
    u32 rc_status;
    u16 link_status;
    u32 link_speed;        /* 1=2.5GT, 2=5.0GT, 3=8.0GT */
    u32 link_width;        /* negotiated lanes */
    u32 ep_count;
    u16 first_vendor;
    u16 first_device;
    u16 b50_vendor;
    u16 b50_device;
    bool b50_found;
    bool scan_truncated;
    const char *fail_reason;
    struct pcie1_ep eps[PCIE1_SCAN_MAX];
};

/* Pure logic — host-tested. */

static inline bool pcie1_is_b50(u16 vendor, u16 device)
{
    return vendor == PCIE1_VID_INTEL && device == PCIE1_DID_B50;
}

static inline bool pcie1_id_valid(u32 cfg0)
{
    return cfg0 != 0U && cfg0 != 0xFFFFFFFFU;
}

static inline u16 pcie1_cfg_vendor(u32 cfg0)
{
    return (u16)(cfg0 & 0xFFFFU);
}

static inline u16 pcie1_cfg_device(u32 cfg0)
{
    return (u16)(cfg0 >> 16);
}

static inline u32 pcie1_link_speed(u16 link_status)
{
    return (u32)(link_status & 0xFU);
}

static inline u32 pcie1_link_width(u16 link_status)
{
    return (u32)((link_status >> 4) & 0x3FU);
}

static inline u8 pcie1_hdr_kind_id(u8 hdr_type)
{
    return (u8)(hdr_type & 0x7FU);
}

static inline bool pcie1_hdr_multifunction(u8 hdr_type)
{
    return (hdr_type & 0x80U) != 0U;
}

static inline bool pcie1_is_bridge(u8 hdr_type)
{
    return pcie1_hdr_kind_id(hdr_type) == 1U;
}

/* Display/3D/processing-accelerator — any of these is a compute candidate.
 * B50 DID counts even if class bytes were not filled. */
static inline bool pcie1_is_compute_class(u8 base, u8 sub)
{
    if (base == 0x03U && (sub == 0x00U || sub == 0x02U))
        return true;
    if (base == 0x12U)
        return true;
    return false;
}

static inline bool pcie1_is_compute_ep(const struct pcie1_ep *e)
{
    if (!e)
        return false;
    if (pcie1_is_b50(e->vendor, e->device))
        return true;
    return pcie1_is_compute_class(e->base_class, e->subclass);
}

static inline const char *pcie1_hdr_kind(u8 hdr_type)
{
    switch (pcie1_hdr_kind_id(hdr_type)) {
    case 0:  return "EP";
    case 1:  return "BR";
    case 2:  return "CB";
    default: return "??";
    }
}

/* Short class label for the dashboard. Not a PCI ID database. */
static inline const char *pcie1_class_label(u8 base, u8 sub)
{
    if (base == 0x01U) {
        if (sub == 0x08U) return "NVMe";
        if (sub == 0x06U) return "SATA";
        return "storage";
    }
    if (base == 0x02U) return "network";
    if (base == 0x03U) {
        if (sub == 0x02U) return "3D GPU";
        if (sub == 0x00U) return "VGA";
        return "display";
    }
    if (base == 0x04U) {
        if (sub == 0x03U) return "HD audio";
        return "multimedia";
    }
    if (base == 0x06U) {
        if (sub == 0x04U || sub == 0x09U) return "PCI bridge";
        if (sub == 0x00U) return "host bridge";
        return "bridge";
    }
    if (base == 0x0CU) {
        if (sub == 0x03U) return "USB";
        if (sub == 0x04U) return "Fibre";
        return "serial bus";
    }
    if (base == 0x0DU) return "wireless";
    if (base == 0x12U) return "accelerator";
    return "other";
}

static inline const char *pcie1_vendor_name(u16 vendor)
{
    switch (vendor) {
    case 0x8086U: return "Intel";
    case 0x1002U: return "AMD";
    case 0x10DEU: return "NVIDIA";
    case 0x10B5U: return "PLX";
    case 0x12D8U: return "Pericom";
    case 0x1B21U: return "ASMedia";
    case 0x14E4U: return "Broadcom";
    case 0x10ECU: return "Realtek";
    case 0x1DE4U: return "RPi";
    case 0x1B36U: return "RedHat";
    case 0x1AF4U: return "virtio";
    default:      return NULL;
    }
}

static inline void pcie1_fill_ep(struct pcie1_ep *e, u8 bus, u8 dev, u8 func,
                                 u32 cfg0, u32 cfg8, u32 cfgc, u32 cfg18)
{
    if (!e)
        return;
    e->bus = bus;
    e->dev = dev;
    e->func = func;
    e->vendor = pcie1_cfg_vendor(cfg0);
    e->device = pcie1_cfg_device(cfg0);
    e->revision = (u8)(cfg8 & 0xFFU);
    e->prog_if = (u8)((cfg8 >> 8) & 0xFFU);
    e->subclass = (u8)((cfg8 >> 16) & 0xFFU);
    e->base_class = (u8)((cfg8 >> 24) & 0xFFU);
    e->hdr_type = (u8)((cfgc >> 16) & 0xFFU);
    if (pcie1_is_bridge(e->hdr_type)) {
        e->sec_bus = (u8)((cfg18 >> 8) & 0xFFU);
        e->sub_bus = (u8)((cfg18 >> 16) & 0xFFU);
    } else {
        e->sec_bus = 0;
        e->sub_bus = 0;
    }
}

static inline char pcie1_hex_digit(u32 v)
{
    v &= 0xFU;
    return (char)(v < 10U ? '0' + v : 'A' + (v - 10U));
}

static inline u32 pcie1_fmt_bdf(char *buf, u32 max, u8 bus, u8 dev, u8 func)
{
    /* "p1 01:00.0" */
    const char prefix[] = "p1 ";
    u32 n = 0;
    u32 i;
    if (!buf || max == 0)
        return 0;
    for (i = 0; prefix[i] && n + 1U < max; i++)
        buf[n++] = prefix[i];
    if (n + 8U < max) {
        buf[n++] = pcie1_hex_digit((u32)bus >> 4);
        buf[n++] = pcie1_hex_digit(bus);
        buf[n++] = ':';
        buf[n++] = pcie1_hex_digit((u32)dev >> 4);
        buf[n++] = pcie1_hex_digit(dev);
        buf[n++] = '.';
        buf[n++] = pcie1_hex_digit(func);
    }
    buf[n] = 0;
    return n;
}

static inline u32 pcie1_fmt_id(char *buf, u32 max, u16 vendor, u16 device)
{
    u32 n = 0;
    if (!buf || max < 10U)
        return 0;
    buf[n++] = pcie1_hex_digit((u32)vendor >> 12);
    buf[n++] = pcie1_hex_digit((u32)vendor >> 8);
    buf[n++] = pcie1_hex_digit((u32)vendor >> 4);
    buf[n++] = pcie1_hex_digit(vendor);
    buf[n++] = ':';
    buf[n++] = pcie1_hex_digit((u32)device >> 12);
    buf[n++] = pcie1_hex_digit((u32)device >> 8);
    buf[n++] = pcie1_hex_digit((u32)device >> 4);
    buf[n++] = pcie1_hex_digit(device);
    buf[n] = 0;
    return n;
}

static inline u32 pcie1_fmt_caps(char *buf, u32 max, const struct pcie1_ep *e)
{
    const char *vn;
    const char *cl;
    u32 n = 0;
    if (!buf || max == 0 || !e)
        return 0;
    vn = pcie1_vendor_name(e->vendor);
    cl = pcie1_class_label(e->base_class, e->subclass);
    if (vn) {
        while (*vn && n + 1U < max)
            buf[n++] = *vn++;
        if (n + 1U < max)
            buf[n++] = ' ';
    }
    while (cl && *cl && n + 1U < max)
        buf[n++] = *cl++;
    if (pcie1_is_bridge(e->hdr_type) && n + 6U < max) {
        buf[n++] = ' ';
        buf[n++] = '-';
        buf[n++] = '>';
        buf[n++] = pcie1_hex_digit((u32)e->sec_bus >> 4);
        buf[n++] = pcie1_hex_digit(e->sec_bus);
        buf[n++] = '-';
        buf[n++] = pcie1_hex_digit((u32)e->sub_bus >> 4);
        buf[n++] = pcie1_hex_digit(e->sub_bus);
    }
    buf[n] = 0;
    return n;
}

/* Memory BAR size from the standard all-1s probe mask. lo/hi already
 * have the low type bits. Fail closed on a zero mask. */
static inline u64 pcie1_bar_size_from_mask(u32 lo, u32 hi, bool is_64)
{
    u32 lo_bits = lo & ~0xFU;
    if (lo_bits == 0U && (!is_64 || hi == 0U))
        return 0ULL;
    if (!is_64)
        return (u64)(~lo_bits + 1U);
    u64 mask = ((u64)hi << 32) | (u64)lo_bits;
    return ~mask + 1ULL;
}

/* CPU outbound windows must not intersect. Empty ranges fail closed. */
static inline bool pcie1_cpu_win_overlaps(u64 a_base, u64 a_size,
                                          u64 b_base, u64 b_size)
{
    if (a_size == 0ULL || b_size == 0ULL)
        return true;
    u64 a_end = a_base + a_size;
    u64 b_end = b_base + b_size;
    if (a_end < a_base || b_end < b_base)
        return true;
    return a_base < b_end && b_base < a_end;
}

bool pcie1_init(void);
bool pcie1_link_up(void);
void pcie1_status(struct pcie1_status *out);
void pcie1_rescan(void);
u32  pcie1_cfg_read(u32 bus, u32 dev, u32 func, u32 reg);
void pcie1_cfg_write(u32 bus, u32 dev, u32 func, u32 reg, u32 val);
