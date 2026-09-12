/*
 * pcie1_bar_lease.c - pure PCIe1 BAR/MMIO authority contract.
 *
 * This validates numeric evidence only. It has no mapping, configuration,
 * MMIO, interrupt, AIRQ, Memory Space, or Bus Master operation.
 */
#include "types.h"
#include "pcie1_bar_lease.h"

#define PCIE1_BAR_LEASE_TOKEN_NONCE_MASK 0x0000FFFFFFFFFFFFULL
#define PCIE1_BAR_LEASE_TOKEN_MASK \
    ((PCIE1_BAR_LEASE_TOKEN_MAGIC << PCIE1_BAR_LEASE_TOKEN_SHIFT) | \
     PCIE1_BAR_LEASE_TOKEN_NONCE_MASK)

static u64 bar_lease_next_capability = 1U;

static inline u64 bar_lease_irq_save(void)
{
#ifdef PIOS_HOST_TYPES_SHIM
     return 0U;
#else
     u64 daif;
     __asm__ volatile("mrs %0, daif" : "=r"(daif));
     __asm__ volatile("msr daifset, #2" ::: "memory");
     return daif;
#endif
}

static inline void bar_lease_irq_restore(u64 daif)
{
#ifdef PIOS_HOST_TYPES_SHIM
     (void)daif;
#else
     __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#endif
}

static bool lease_fresh(const struct pcie1_bar_lease *lease)
{
    const u8 *bytes;
    usize i;

    if (!lease)
        return false;
    bytes = (const u8 *)lease;
    for (i = 0U; i < sizeof(*lease); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
}

static bool range_valid(u64 base, u64 length, u64 *end_out)
{
    if (length == 0U || base > ~0ULL - length)
        return false;
    if (end_out)
        *end_out = base + length;
    return true;
}

static bool ranges_overlap(u64 a_base, u64 a_length, u64 b_base,
                           u64 b_length)
{
    u64 a_end;
    u64 b_end;

    if (!range_valid(a_base, a_length, &a_end) ||
        !range_valid(b_base, b_length, &b_end))
        return true;
    return a_base < b_end && b_base < a_end;
}

static bool power_of_two(u64 value)
{
    return value != 0U && (value & (value - 1U)) == 0U;
}

static bool bdf_valid(u32 bdf)
{
    return (bdf & ~0xFFFFU) == 0U && ((bdf >> 8) & 0xFFU) != 0U;
}

static bool owner_core(u32 caller_core)
{
    return caller_core == PCIE1_BAR_LEASE_OWNER_CORE &&
           core_id() == PCIE1_BAR_LEASE_OWNER_CORE;
}

static bool capability_next(u64 *capability_out)
{
    u64 value;

    if (!capability_out || bar_lease_next_capability == 0U ||
        bar_lease_next_capability > PCIE1_BAR_LEASE_TOKEN_NONCE_MASK)
        return false;
    value = bar_lease_next_capability++;
    *capability_out = value;
    return true;
}

static u64 lease_token(u64 capability)
{
    return (PCIE1_BAR_LEASE_TOKEN_MAGIC << PCIE1_BAR_LEASE_TOKEN_SHIFT) |
           capability;
}

static void clear_handle(struct pcie1_bar_lease_handle *handle)
{
    if (handle)
        *handle = (struct pcie1_bar_lease_handle){0};
}

static void clear_payload(struct pcie1_bar_lease *lease)
{
    u32 i;

    lease->mapping = (struct pcie1_bar_lease_mapping){0};
    for (i = 0U; i < PCIE1_BAR_LEASE_RESERVATION_CAPACITY; i++)
        lease->reservations[i] = (struct pcie1_bar_lease_reservation){0};
}

static bool handle_valid(const struct pcie1_bar_lease *lease,
                         const struct pcie1_bar_lease_handle *handle,
                         u32 caller_core)
{
    return lease && handle &&
           owner_core(caller_core) &&
           lease->control.owner_core == PCIE1_BAR_LEASE_OWNER_CORE &&
           lease->control.contract_id != 0U &&
           handle->contract_id == lease->control.contract_id &&
           handle->bdf == lease->control.bdf &&
           handle->generation != 0U &&
           handle->generation == lease->control.generation &&
           handle->_reserved == 0U &&
           (handle->token & ~PCIE1_BAR_LEASE_TOKEN_MASK) == 0U &&
           handle->token == lease_token(lease->control.capability);
}

static bool handle_active(const struct pcie1_bar_lease *lease,
                          const struct pcie1_bar_lease_handle *handle,
                          u32 caller_core)
{
    return handle_valid(lease, handle, caller_core) &&
           lease->control.state == PCIE1_BAR_LEASE_ACTIVE &&
           lease->control.fault == PCIE1_BAR_LEASE_FAULT_NONE;
}

static bool bar_decode(const struct pcie1_bar_lease_bar *bar,
                       u64 *base_out, u64 *size_out, bool *is_64_out)
{
    u32 type;
    u32 lo_mask;
    u64 base;
    u64 mask;
    u64 size;
    bool is_64;

    if (!bar || !base_out || !size_out || !is_64_out ||
        (bar->config_lo & 1U) != 0U ||
        (bar->config_lo & 8U) != 0U)
        return false;
    type = (bar->config_lo >> 1) & 3U;
    if (type != 0U && type != 2U)
        return false;
    is_64 = type == 2U;
    if (!is_64 && (bar->config_hi != 0U || bar->probe_hi != 0U))
        return false;
    if ((bar->probe_lo & 0xFU) != (bar->config_lo & 0xFU))
        return false;

    lo_mask = bar->probe_lo & ~0xFU;
    if (!is_64) {
        if (lo_mask == 0U)
            return false;
        size = (u64)(~lo_mask + 1U);
        if (!power_of_two(size) ||
            lo_mask != (~((u32)size - 1U) & ~0xFU))
            return false;
        base = (u64)(bar->config_lo & ~0xFU);
    } else {
        mask = ((u64)bar->probe_hi << 32) | (u64)lo_mask;
        if (mask == 0U)
            return false;
        size = ~mask + 1U;
        if (!power_of_two(size) || mask != ~(size - 1U))
            return false;
        base = ((u64)bar->config_hi << 32) |
               (u64)(bar->config_lo & ~0xFU);
    }
    if (!range_valid(base, size, NULL) || (base & (size - 1U)) != 0U ||
        (!is_64 && base > 0x100000000ULL - size))
        return false;
    *base_out = base;
    *size_out = size;
    *is_64_out = is_64;
    return true;
}

static bool reservation_set_valid(
    const struct pcie1_bar_lease_reservation *reservations, u32 count,
    u64 cpu_base, u64 pci_base, u64 length)
{
    u32 i;

    if (count > PCIE1_BAR_LEASE_RESERVATION_CAPACITY ||
        (count != 0U && !reservations))
        return false;
    if (ranges_overlap(cpu_base, length, PIOS_RP1_BAR_BASE,
                       PCIE1_BAR_LEASE_RP1_BYTES))
        return false;
    for (i = 0U; i < count; i++) {
        const struct pcie1_bar_lease_reservation *reservation =
            &reservations[i];

        if (!range_valid(reservation->base, reservation->length, NULL) ||
            (reservation->domain != PCIE1_BAR_LEASE_DOMAIN_CPU_PHYS &&
             reservation->domain != PCIE1_BAR_LEASE_DOMAIN_PCI) ||
            (reservation->domain == PCIE1_BAR_LEASE_DOMAIN_CPU_PHYS &&
             ranges_overlap(cpu_base, length, reservation->base,
                            reservation->length)) ||
            (reservation->domain == PCIE1_BAR_LEASE_DOMAIN_PCI &&
             ranges_overlap(pci_base, length, reservation->base,
                            reservation->length)))
            return false;
    }
    return true;
}

static bool request_valid(const struct pcie1_bar_lease_request *request,
                          u64 *bar_base_out, u64 *bar_size_out,
                          bool *is_64_out)
{
    u64 window_end;
    u64 bar_base;
    u64 bar_size;
    bool is_64;

    if (!request || request->_reserved != 0U ||
        request->aperture._reserved != 0U ||
        request->aperture.attribute != PCIE1_BAR_LEASE_ATTR_DEVICE_nGnRnE ||
        !bar_decode(&request->bar, &bar_base, &bar_size, &is_64) ||
        request->aperture.length != bar_size ||
        request->aperture.pci_base != bar_base ||
        !range_valid(request->aperture.cpu_base, request->aperture.length,
                     NULL) ||
        !range_valid(PIOS_PCIE1_CPU_WIN_BASE, PIOS_PCIE1_CPU_WIN_SIZE,
                     &window_end) ||
        request->aperture.cpu_base < PIOS_PCIE1_CPU_WIN_BASE ||
        request->aperture.cpu_base >= window_end ||
        request->aperture.length >
        window_end - request->aperture.cpu_base ||
        !reservation_set_valid(request->reservations,
                               request->reservation_count,
                               request->aperture.cpu_base, bar_base,
                               bar_size))
        return false;
    *bar_base_out = bar_base;
    *bar_size_out = bar_size;
    *is_64_out = is_64;
    return true;
}

static void retire_or_free(struct pcie1_bar_lease *lease)
{
    clear_payload(lease);
    if (lease->control.generation == ~0ULL) {
        lease->control.generation = 0U;
        lease->control.exhausted = 1U;
        lease->control.state = PCIE1_BAR_LEASE_RETIRED;
        return;
    }
    lease->control.generation++;
    lease->control.state = PCIE1_BAR_LEASE_FREE;
}

static bool bar_lease_init_locked(struct pcie1_bar_lease *lease, u32 contract_id,
                          u32 caller_core)
{
    if (!lease || contract_id == 0U || !owner_core(caller_core) ||
        !PIOS_HAS_PCIE1 ||
        !lease_fresh(lease))
        return false;
    lease->control.generation = 1U;
    lease->control.contract_id = contract_id;
    lease->control.owner_core = PCIE1_BAR_LEASE_OWNER_CORE;
    lease->control.state = PCIE1_BAR_LEASE_FREE;
    return true;
}

static bool bar_lease_acquire_locked(
    struct pcie1_bar_lease *lease, u32 bdf, u32 caller_core,
    const struct pcie1_bar_lease_request *request,
    struct pcie1_bar_lease_handle *handle_out)
{
    u64 bar_base;
    u64 bar_size;
    u64 capability;
    bool is_64;
    u32 i;

    clear_handle(handle_out);
    if (!lease || !handle_out || !bdf_valid(bdf) ||
        !owner_core(caller_core) ||
        lease->control.owner_core != PCIE1_BAR_LEASE_OWNER_CORE ||
        lease->control.contract_id == 0U ||
        lease->control.state != PCIE1_BAR_LEASE_FREE ||
        lease->control.exhausted != 0U ||
        !request_valid(request, &bar_base, &bar_size, &is_64) ||
        !capability_next(&capability))
        return false;

    lease->control.bdf = bdf;
    lease->control.capability = capability;
    lease->control.fault = PCIE1_BAR_LEASE_FAULT_NONE;
    lease->mapping.bar_pci_base = bar_base;
    lease->mapping.bar_size = bar_size;
    lease->mapping.cpu_base = request->aperture.cpu_base;
    lease->mapping.aperture_pci_base = request->aperture.pci_base;
    lease->mapping.aperture_length = request->aperture.length;
    lease->mapping.bar_is_64 = is_64 ? 1U : 0U;
    lease->mapping.attribute = request->aperture.attribute;
    lease->mapping.reservation_count = request->reservation_count;
    for (i = 0U; i < request->reservation_count; i++)
        lease->reservations[i] = request->reservations[i];
    lease->control.state = PCIE1_BAR_LEASE_ACTIVE;
    handle_out->token = lease_token(capability);
    handle_out->generation = lease->control.generation;
    handle_out->contract_id = lease->control.contract_id;
    handle_out->bdf = bdf;
    return true;
}

static bool bar_lease_release_locked(
    struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core)
{
    if (!handle_active(lease, handle, caller_core))
        return false;
    retire_or_free(lease);
    return true;
}

static bool bar_lease_revoke_locked(
    struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core,
    enum pcie1_bar_lease_fault fault)
{
    if (!handle_active(lease, handle, caller_core) ||
        (fault != PCIE1_BAR_LEASE_FAULT_AER &&
         fault != PCIE1_BAR_LEASE_FAULT_REMOVED &&
         fault != PCIE1_BAR_LEASE_FAULT_FAILURE))
        return false;
    clear_payload(lease);
    lease->control.capability = 0U;
    lease->control.revoke_count++;
    lease->control.fault = (u32)fault;
    if (lease->control.generation == ~0ULL) {
        lease->control.generation = 0U;
        lease->control.exhausted = 1U;
        lease->control.state = PCIE1_BAR_LEASE_RETIRED;
    } else {
        lease->control.generation++;
        lease->control.state = fault == PCIE1_BAR_LEASE_FAULT_REMOVED ?
            PCIE1_BAR_LEASE_REMOVED : PCIE1_BAR_LEASE_QUARANTINED;
    }
    return true;
}

static bool bar_lease_mapping_get_locked(
    const struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core,
    struct pcie1_bar_lease_mapping *mapping_out)
{
    if (mapping_out)
        *mapping_out = (struct pcie1_bar_lease_mapping){0};
    if (!mapping_out || !handle_active(lease, handle, caller_core))
        return false;
    *mapping_out = lease->mapping;
    return true;
}

static bool bar_lease_state_get_locked(
    const struct pcie1_bar_lease *lease, u32 caller_core,
    enum pcie1_bar_lease_state *state_out,
    enum pcie1_bar_lease_fault *fault_out)
{
    if (state_out)
        *state_out = PCIE1_BAR_LEASE_RETIRED;
    if (fault_out)
        *fault_out = PCIE1_BAR_LEASE_FAULT_NONE;
    if (!lease || !state_out || !fault_out ||
        !owner_core(caller_core) ||
        lease->control.owner_core != PCIE1_BAR_LEASE_OWNER_CORE ||
        lease->control.contract_id == 0U)
        return false;
    *state_out = (enum pcie1_bar_lease_state)lease->control.state;
    *fault_out = (enum pcie1_bar_lease_fault)lease->control.fault;
    return true;
}

bool pcie1_bar_lease_init(struct pcie1_bar_lease *lease, u32 contract_id,
                          u32 caller_core)
{
    u64 irq = bar_lease_irq_save();
    bool ok = bar_lease_init_locked(lease, contract_id, caller_core);
    bar_lease_irq_restore(irq);
    return ok;
}

bool pcie1_bar_lease_acquire(
    struct pcie1_bar_lease *lease, u32 bdf, u32 caller_core,
    const struct pcie1_bar_lease_request *request,
    struct pcie1_bar_lease_handle *handle_out)
{
    u64 irq = bar_lease_irq_save();
    bool ok = bar_lease_acquire_locked(lease, bdf, caller_core, request,
                                       handle_out);
    bar_lease_irq_restore(irq);
    return ok;
}

bool pcie1_bar_lease_release(
    struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core)
{
    u64 irq = bar_lease_irq_save();
    bool ok = bar_lease_release_locked(lease, handle, caller_core);
    bar_lease_irq_restore(irq);
    return ok;
}

bool pcie1_bar_lease_revoke(
    struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core,
    enum pcie1_bar_lease_fault fault)
{
    u64 irq = bar_lease_irq_save();
    bool ok = bar_lease_revoke_locked(lease, handle, caller_core, fault);
    bar_lease_irq_restore(irq);
    return ok;
}

bool pcie1_bar_lease_mapping_get(
    const struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core,
    struct pcie1_bar_lease_mapping *mapping_out)
{
    u64 irq = bar_lease_irq_save();
    bool ok = bar_lease_mapping_get_locked(lease, handle, caller_core,
                                           mapping_out);
    bar_lease_irq_restore(irq);
    return ok;
}

bool pcie1_bar_lease_state_get(
    const struct pcie1_bar_lease *lease, u32 caller_core,
    enum pcie1_bar_lease_state *state_out,
    enum pcie1_bar_lease_fault *fault_out)
{
    u64 irq = bar_lease_irq_save();
    bool ok = bar_lease_state_get_locked(lease, caller_core, state_out,
                                         fault_out);
    bar_lease_irq_restore(irq);
    return ok;
}

bool pcie1_bar_lease_hardware_enable_allowed(
    const struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core)
{
    (void)lease;
    (void)handle;
    (void)caller_core;
    return false;
}
