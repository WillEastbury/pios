#include "types.h"
#include "b50_native.h"
#include "pcie1.h"
#include "pcie1_bar_lease.h"
#include "pcie1_containment.h"
#include "lzero.h"
#include "mmu.h"
#ifndef B50_NATIVE_HOST_TEST
#include "mmio.h"
#else
u32 b50_native_test_read(u64 addr);
void *b50_native_test_pointer(u64 addr);
#define mmio_read b50_native_test_read
#endif
#include "timer.h"
#include "gic.h"
#include "exception.h"
#include "uart.h"

/* All lifecycle mutation is core-0 scheduled context, never IRQ context. */
#if PIOS_HAS_PCIE1
static struct pcie1_bar_lease lease;
static struct pcie1_containment containment;
static struct {
    struct pcie1_bar_lease_handle bar;
    struct pcie1_containment_endpoint_handle endpoint;
} ALIGNED(64) authority;
static struct {
    u32 initialized;
    u32 attached;
    u8 _pad[56];
} ALIGNED(64) native;

#define B50_ROOT_MSI_INTID 256U
static struct {
    volatile u64 sequence;
    u64 deadline;
    u8 _pad[48];
} ALIGNED(64) trigger;
static struct {
    u64 acknowledged;
    u64 queued;
    u8 _pad[48];
} ALIGNED(64) continuation;

/* Decode remains disabled. Any interrupt before a real engine exists is
 * unexpected and can only revoke authority, never certify a completion. */
static void native_irq(void)
{
    gic_disable_irq(B50_ROOT_MSI_INTID);
    trigger.deadline = timer_monotonic_ms() + 100U;
    u64 sequence = trigger.sequence + 1U;
    if (!sequence) sequence = ~0ULL;
    dmb();
    trigger.sequence = sequence;
    (void)airq_post_from(0U, AIRQ_SRC_PCIE1_MSI, 0U);
}

static const char *quarantine(void);

static void native_bottom(const struct airq_record *record, void *ctx)
{
    (void)ctx;
    if (!record || core_id() || record->origin_core || record->target_core)
        return;
    u64 sequence = trigger.sequence;
    dmb();
    if (sequence == continuation.acknowledged)
        return;
    uart_puts(quarantine());
    continuation.acknowledged = sequence;
}

static const u32 buses[] = {1U, 2U, 3U, 4U, 5U, 9U, 13U};
static const u32 devices[] = {0U, 8U, 0U, 1U, 0U, 0U, 0U};
static const u32 identities[] = {
    0x874810B5U, 0x874810B5U, 0xE2FF8086U, 0xE2F08086U,
    0xE2128086U, 0xE2128086U, 0xE2128086U
};

static bool revoke_bme(void)
{
    bool ok = true;
    for (u32 i = 0; i < 5U; i++) {
        if (pcie1_cfg_read(buses[i], devices[i], 0U, 0U) != identities[i]) {
            ok = false;
            continue;
        }
        u32 command = pcie1_cfg_read(buses[i], devices[i], 0U, 4U);
        if (command == ~0U) {
            ok = false;
            continue;
        }
        /* Never echo PCI status W1C bits in the upper halfword. */
        pcie1_cfg_write(buses[i], devices[i], 0U, 4U,
                        (command & 0xFFFFU) & ~4U);
        if (pcie1_cfg_read(buses[i], devices[i], 0U, 4U) & 4U)
            ok = false;
    }
    dsb();
    return ok;
}

static bool live_valid(void)
{
    u64 deadline = timer_monotonic_ms() + 50U;
    struct lzero_status status;
    struct pcie1_status topology;
    struct pcie1_aer_snapshot aer;
    static const u32 offsets[] = {
        0x402CU, 0x4030U, 0x4034U, 0x4038U, 0x403CU, 0x4040U,
        0x4044U, 0x4048U, 0x40ACU, 0x40B0U, 0x40B4U, 0x40B8U,
        0x40BCU, 0x40C0U, 0x400CU, 0x4010U, 0x4070U, 0x4080U,
        0x4084U, 0x20U
    };
    static const u32 values[] = {
        0U, 0U, 6U, 0x10U, 0U, 0U, 0U, 0U, 0U, 0U,
        (u32)PIOS_DMA_PCIE1_BASE | 1U, 0U, 0U, 0U,
        0x80000000U, 0U, 0x80F08000U, 0x1BU, 0x1BU, 0x80F08000U
    };
    if (!pcie1_link_up())
        return false;
    pcie1_status(&topology);
    if (topology.scan_truncated || topology.malformed_topology ||
        topology.ep_count > PCIE1_SCAN_MAX)
        return false;
    for (u32 i = 0; i < topology.ep_count; i++) {
        const struct pcie1_ep *ep = &topology.eps[i];
        u32 id = pcie1_cfg_read(ep->bus, ep->dev, ep->func, 0U);
        if (id != ((u32)ep->device << 16 | ep->vendor) ||
            (pcie1_cfg_read(ep->bus, ep->dev, ep->func, 4U) & 4U))
            return false;
        /* Bounded extended-capability walk; never clear AER evidence. */
        u32 off = 0x100U;
        for (u32 steps = 0U; off && steps < 64U; steps++) {
            u32 header = pcie1_cfg_read(ep->bus, ep->dev, ep->func, off);
            if (!header) { off = 0U; break; }
            if (header == ~0U) return false;
            if ((header & 0xFFFFU) == 1U) {
                if (off > 0xFECU ||
                    pcie1_cfg_read(ep->bus, ep->dev, ep->func, off + 4U) ||
                    pcie1_cfg_read(ep->bus, ep->dev, ep->func, off + 16U))
                    return false;
            }
            u32 next = header >> 20;
            if (next && (next <= off || next > 0xFFCU || (next & 3U)))
                return false;
            off = next;
            if (timer_monotonic_ms() >= deadline) return false;
        }
        if (off) return false;
    }
    for (u32 i = 0; i < 7U; i++) {
        if (pcie1_cfg_read(buses[i], devices[i], 0U, 0U) != identities[i] ||
            (pcie1_cfg_read(buses[i], devices[i], 0U, 4U) & 7U) !=
                (i < 5U ? 2U : 0U))
            return false;
        if (i < 4U) {
            u32 range = pcie1_cfg_read(buses[i], devices[i], 0U, 0x18U);
            if (((range >> 8) & 255U) > 5U ||
                ((range >> 16) & 255U) < 5U ||
                pcie1_cfg_read(buses[i], devices[i], 0U, 0x20U) != 0x80F08000U)
                return false;
        }
    }
    lzero_status(&status);
    if (!status.bars_probed || !status.bar0_mapped ||
        status.gpu_bus != 5U || status.gpu_dev || status.gpu_func ||
        status.bar0_size != 0x1000000ULL ||
        pcie1_cfg_read(5U, 0U, 0U, 0x10U) != 0x80000004U ||
        pcie1_cfg_read(5U, 0U, 0U, 0x14U) != 0U)
        return false;
    for (u32 i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        u64 addr = PIOS_PCIE1_RC_BASE + offsets[i];
        if (!mmu_device_read32_valid(addr) || mmio_read(addr) != values[i])
            return false;
    }
    if (!mmu_device_read32_valid(PIOS_PCIE1_RC_BASE + 0x4008U) ||
        ((mmio_read(PIOS_PCIE1_RC_BASE + 0x4008U) >> 27) & 31U) != 6U)
        return false;
    pcie1_aer_snapshot(&aer, false);
    if (!aer.aer_offset || aer.uncorr || aer.corr ||
        !mmu_active_nc_range_valid(PIOS_DMA_PCIE1_BASE, PIOS_DMA_PCIE1_SIZE))
        return false;
    for (u64 off = 0; off < 0x1000000ULL; off += 4096U)
        if (timer_monotonic_ms() >= deadline ||
            !mmu_device_read32_valid(PIOS_PCIE1_CPU_WIN_BASE + off))
            return false;
    for (u32 i = 0; i < 3U; i++)
        if (mmio_read(PIOS_PCIE1_CPU_WIN_BASE + 0xD8CU) != 0x05004000U)
            return false;
    return timer_monotonic_ms() < deadline;
}

static bool owned_slice_check(void)
{
    struct pcie1_containment_dma_handle handle;
    struct pcie1_containment_dma_span span;
    u64 now = timer_monotonic_ms();
    u64 request = containment.owner.last_request_id + 1U;
    if (!request || !pcie1_containment_dma_acquire(
            &containment, &authority.endpoint, 0U, request, 64U,
            PCIE1_CONTAINMENT_FROM_DEVICE, now, 50U, &handle))
        return false;
    if (!pcie1_containment_dma_span_get(&containment, &handle, 0U, &span) ||
        span.payload_cpu_phys < PIOS_DMA_PCIE1_BASE + 64U ||
        span.capacity != PCIE1_CONTAINMENT_DMA_PAYLOAD_BYTES ||
        span.payload_cpu_phys > PIOS_DMA_PCIE1_BASE + PIOS_DMA_PCIE1_SIZE -
                               span.capacity - 64U)
        return false;
    u64 base = span.payload_cpu_phys - 64U;
#ifndef B50_NATIVE_HOST_TEST
    volatile u8 *bytes = (volatile u8 *)(usize)base;
#else
    volatile u8 *bytes = b50_native_test_pointer(base);
#endif
    for (u32 i = 0; i < 64U; i++) {
        bytes[i] = 0xA5U;
        bytes[64U + span.capacity + i] = 0x5AU;
        bytes[64U + i] = (u8)(i ^ handle.generation);
    }
    dsb();
    bool intact = true;
    for (u32 i = 0; i < 64U; i++)
        if (bytes[i] != 0xA5U || bytes[64U + span.capacity + i] != 0x5AU ||
            bytes[64U + i] != (u8)(i ^ handle.generation))
            intact = false;
    /* Only the requested payload and its guards were touched. */
    for (u32 i = 0; i < 64U; i++) {
        bytes[i] = bytes[64U + i] = 0xDDU;
        bytes[64U + span.capacity + i] = 0xDDU;
    }
    dsb();
    return intact &&
        pcie1_containment_dma_cancel(&containment, &authority.endpoint, &handle, 0U) &&
        pcie1_containment_dma_release(&containment, &authority.endpoint, &handle, 0U);
}

static bool probe_bar(struct pcie1_bar_lease_bar *bar)
{
    u32 command = pcie1_cfg_read(5U, 0U, 0U, 4U);
    bar->config_lo = pcie1_cfg_read(5U, 0U, 0U, 0x10U);
    bar->config_hi = pcie1_cfg_read(5U, 0U, 0U, 0x14U);
    if ((command & 7U) != 2U || bar->config_lo != 0x80000004U ||
        bar->config_hi != 0U)
        return false;
    pcie1_cfg_write(5U, 0U, 0U, 4U, (command & 0xFFFFU) & ~7U);
    if (pcie1_cfg_read(5U, 0U, 0U, 4U) & 7U)
        return false;
    pcie1_cfg_write(5U, 0U, 0U, 0x10U, ~0U);
    pcie1_cfg_write(5U, 0U, 0U, 0x14U, ~0U);
    bar->probe_lo = pcie1_cfg_read(5U, 0U, 0U, 0x10U);
    bar->probe_hi = pcie1_cfg_read(5U, 0U, 0U, 0x14U);
    pcie1_cfg_write(5U, 0U, 0U, 0x14U, bar->config_hi);
    pcie1_cfg_write(5U, 0U, 0U, 0x10U, bar->config_lo);
    dsb();
    if (pcie1_cfg_read(5U, 0U, 0U, 0x10U) != bar->config_lo ||
        pcie1_cfg_read(5U, 0U, 0U, 0x14U) != bar->config_hi)
        return false;
    pcie1_cfg_write(5U, 0U, 0U, 4U, command & 0xFFFFU);
    return (pcie1_cfg_read(5U, 0U, 0U, 4U) & 7U) == 2U;
}

static const char *quarantine(void)
{
    gic_disable_irq(B50_ROOT_MSI_INTID);
    bool revoked = revoke_bme();
    if (native.attached) {
        (void)pcie1_containment_quarantine(&containment, &authority.endpoint, 0U,
                                          PCIE1_CONTAINMENT_FAULT_AER);
        (void)pcie1_bar_lease_revoke(&lease, &authority.bar, 0U,
                                    PCIE1_BAR_LEASE_FAULT_FAILURE);
        native.attached = 0U;
    }
    return revoked ? "b50 quarantined; BME revoked; cold recovery required\n" :
                     "b50 quarantined; BME revocation UNCONFIRMED; cold recovery required\n";
}
#endif

void b50_native_service(void)
{
#if PIOS_HAS_PCIE1
    if (core_id() || !native.initialized)
        return;
    u64 sequence = trigger.sequence;
    dmb();
    if (sequence == continuation.acknowledged)
        return;
    if (timer_monotonic_ms() >= trigger.deadline) {
        uart_puts(quarantine());
        continuation.acknowledged = sequence;
        return;
    }
    if (continuation.queued != sequence &&
        airq_post_from(0U, AIRQ_SRC_PCIE1_MSI, 0U))
        continuation.queued = sequence;
#endif
}

bool b50_native_blocks_legacy(void)
{
#if PIOS_HAS_PCIE1
    return native.initialized != 0U;
#else
    return false;
#endif
}

const char *b50_native_command(u32 operation)
{
    if (core_id() != 0U)
        return "b50 rejected: core-0 owner only\n";
#if !PIOS_HAS_PCIE1
    (void)operation;
    return "b50 rejected: platform has no PCIe1\n";
#else
    if (operation == B50_NATIVE_STATUS)
        return native.attached ?
            "b50 attached; BME enable unavailable; queue/firmware/IRQ unavailable; DMA unproven\n" :
            "b50 detached; no hardware authority; DMA unproven\n";
    if (operation == B50_NATIVE_REVOKE)
        return quarantine();
    if (!live_valid())
        return quarantine();
    if (operation == B50_NATIVE_ATTACH) {
        struct pcie1_bar_lease_request request = {0};
        if (native.attached)
            return "b50 already attached\n";
        if (native.initialized)
            return "b50 rejected: retired lifecycle; cold recovery required\n";
        native.initialized = 1U;
        gic_disable_irq(B50_ROOT_MSI_INTID);
        if (!airq_register(AIRQ_SRC_PCIE1_MSI, AIRQ_PRIO_NORMAL, 0U,
                            native_bottom, NULL))
            return quarantine();
        irq_register(B50_ROOT_MSI_INTID, native_irq);
        if (!probe_bar(&request.bar))
            return quarantine();
        request.aperture.cpu_base = PIOS_PCIE1_CPU_WIN_BASE;
        request.aperture.pci_base = 0x80000000ULL;
        request.aperture.length = 0x1000000ULL;
        request.aperture.attribute = PCIE1_BAR_LEASE_ATTR_DEVICE_nGnRnE;
        if (!pcie1_bar_lease_init(&lease, 82U, 0U) ||
            !pcie1_bar_lease_acquire(&lease, 0x500U, 0U, &request, &authority.bar))
            return quarantine();
        native.attached = 1U;
        if (!pcie1_containment_init(&containment, 82U, 0x500U,
                                    authority.bar.generation, 0U, &authority.endpoint))
            return quarantine();
        return "b50 native generation lease attached; BME remains off\n";
    }
    if (!native.attached)
        return "b50 rejected: attach required\n";
    if (operation == B50_NATIVE_PREFLIGHT) {
        if (!owned_slice_check())
            return quarantine();
        return "b50 mapping/identity/aperture and CPU-owned redzones passed; DMA/IRQ unproven\n";
    }
    if (operation == B50_NATIVE_CANARY)
        return "b50 rejected: bounded hardware queue, GuC firmware and dedicated completion IRQ not implemented; BME remains off\n";
    return "b50 rejected: unknown operation\n";
#endif
}
