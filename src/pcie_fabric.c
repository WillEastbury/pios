#include "pcie_fabric.h"

static bool fabric_bdf_valid(const struct pcie_fabric_function *fn)
{
    return fn && fn->bus < PCIE_FABRIC_MAX_BUSES && fn->dev < 32U &&
           fn->func < 8U && fn->vendor != 0U && fn->vendor != 0xFFFFU;
}

static bool fabric_duplicate(const struct pcie_fabric_snapshot *out,
                             const struct pcie_fabric_function *fn)
{
    for (u32 i = 0; i < out->function_count; i++) {
        const struct pcie_fabric_function *old = &out->functions[i];
        if (old->bus == fn->bus && old->dev == fn->dev && old->func == fn->func)
            return true;
    }
    return false;
}

bool pcie_fabric_observe(u32 root_bus,
                          const struct pcie_fabric_function *observed,
                          u32 observed_count,
                          struct pcie_fabric_snapshot *out)
{
    bool reachable[PCIE_FABRIC_MAX_BUSES] = {0};
    if (!out || !observed || observed_count == 0U ||
        root_bus >= PCIE_FABRIC_MAX_BUSES)
        return false;
    *out = (struct pcie_fabric_snapshot){0};
    out->root_bus = root_bus;
    reachable[root_bus] = true;

    for (u32 pass = 0; pass < PCIE_FABRIC_MAX_BUSES; pass++) {
        bool advanced = false;
        for (u32 i = 0; i < observed_count; i++) {
            const struct pcie_fabric_function *fn = &observed[i];
            if (!fabric_bdf_valid(fn) || !reachable[fn->bus] ||
                !pcie_fabric_is_bridge(fn))
                continue;
            if (fn->secondary_bus <= fn->bus ||
                fn->secondary_bus > fn->subordinate_bus ||
                fn->subordinate_bus >= PCIE_FABRIC_MAX_BUSES) {
                out->malformed_topology = 1U;
                continue;
            }
            for (u32 bus = fn->secondary_bus; bus <= fn->subordinate_bus; bus++) {
                if (!reachable[bus]) {
                    reachable[bus] = true;
                    advanced = true;
                }
            }
        }
        if (!advanced)
            break;
    }

    for (u32 i = 0; i < observed_count; i++) {
        const struct pcie_fabric_function *fn = &observed[i];
        if (!fabric_bdf_valid(fn)) {
            out->malformed_topology = 1U;
            continue;
        }
        if (fabric_duplicate(out, fn)) {
            out->duplicate_bdf = 1U;
            continue;
        }
        if (out->function_count == PCIE_FABRIC_MAX_FUNCTIONS) {
            out->truncated = 1U;
            continue;
        }
        struct pcie_fabric_function *slot = &out->functions[out->function_count++];
        *slot = *fn;
        slot->reachable = reachable[fn->bus] ? 1U : 0U;
        if (!slot->reachable)
            continue;
        if (pcie_fabric_is_bridge(slot))
            continue;
        if (pcie_fabric_is_compute(slot))
            out->compute_count++;
        if (pcie_fabric_is_b50(slot))
            out->b50_count++;
    }
    return out->function_count != 0U && !out->duplicate_bdf;
}
