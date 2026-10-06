/*
 * pcie_fabric.h - architecture-neutral, passive PCIe hierarchy model.
 *
 * This records only bounded config-space observations. It does not configure
 * bridge bus ranges, decode BARs, enable Memory Space/Bus Master, DMA or MSI.
 */
#pragma once
#include "types.h"

#define PCIE_FABRIC_MAX_FUNCTIONS 32U
#define PCIE_FABRIC_MAX_BUSES     64U

#define PCIE_FABRIC_VENDOR_INTEL  0x8086U
#define PCIE_FABRIC_DEVICE_B50    0xE212U

struct pcie_fabric_function {
    u8 bus;
    u8 dev;
    u8 func;
    u8 hdr_type;
    u16 vendor;
    u16 device;
    u8 revision;
    u8 prog_if;
    u8 subclass;
    u8 base_class;
    u8 secondary_bus;
    u8 subordinate_bus;
    u8 reachable;
    u8 _pad[3];
} ALIGNED(32);

struct pcie_fabric_snapshot {
    u32 root_bus;
    u32 function_count;
    u32 compute_count;
    u32 b50_count;
    u32 malformed_topology;
    u32 duplicate_bdf;
    u32 truncated;
    u32 _reserved;
    struct pcie_fabric_function functions[PCIE_FABRIC_MAX_FUNCTIONS];
} ALIGNED(64);

_Static_assert(sizeof(struct pcie_fabric_function) == 32U,
               "fabric function record must retain its fixed stride");
_Static_assert((sizeof(struct pcie_fabric_snapshot) & 63U) == 0U,
               "fabric snapshot must retain a cache-line multiple");

static inline bool pcie_fabric_is_bridge(const struct pcie_fabric_function *fn)
{
    return fn && (fn->hdr_type & 0x7FU) == 1U;
}

static inline bool pcie_fabric_is_b50(const struct pcie_fabric_function *fn)
{
    return fn && fn->vendor == PCIE_FABRIC_VENDOR_INTEL &&
           fn->device == PCIE_FABRIC_DEVICE_B50;
}

static inline bool pcie_fabric_is_compute(const struct pcie_fabric_function *fn)
{
    return fn && (pcie_fabric_is_b50(fn) ||
                  (fn->base_class == 0x03U &&
                   (fn->subclass == 0x00U || fn->subclass == 0x02U)) ||
                  fn->base_class == 0x12U);
}

/* Input entries are immutable config-space observations. Any record outside
 * the established bridge hierarchy is retained as unreachable evidence but
 * never counted as an endpoint eligible for later authority phases. */
bool pcie_fabric_observe(u32 root_bus,
                          const struct pcie_fabric_function *observed,
                          u32 observed_count,
                          struct pcie_fabric_snapshot *out);
