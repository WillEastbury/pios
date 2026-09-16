/*
 * pcie1_containment.h - offline PCIe1 MSI and inbound-DMA containment.
 *
 * This contract owns only numeric authority and lifecycle state. It performs
 * no MMIO, mapping, cache maintenance, IRQ registration, AIRQ publication, or
 * bus-master enablement. A future hardware adapter must supply the endpoint
 * lease created by #188 and remain disabled until the live #189 brick test.
 */
#pragma once

#include "types.h"
#include "platform.h"
#include "airq.h"

#define PCIE1_CONTAINMENT_OWNER_CORE          0U
#define PCIE1_CONTAINMENT_DMA_SLOT_COUNT      8U
#define PCIE1_CONTAINMENT_DMA_SLOT_BYTES      0x00040000U
#define PCIE1_CONTAINMENT_DMA_GUARD_BYTES     64U
#define PCIE1_CONTAINMENT_DMA_PAYLOAD_BYTES   \
    (PCIE1_CONTAINMENT_DMA_SLOT_BYTES -       \
     2U * PCIE1_CONTAINMENT_DMA_GUARD_BYTES)
#define PCIE1_CONTAINMENT_DMA_ALIGNMENT       64U
#define PCIE1_CONTAINMENT_IOVA_BASE           0x1000000000ULL
#define PCIE1_CONTAINMENT_TIMEOUT_MAX_MS      60000ULL
#define PCIE1_CONTAINMENT_AIRQ_SOURCE         AIRQ_SRC_PCIE1_MSI

#define PCIE1_CONTAINMENT_ENDPOINT_MAGIC      0xC189ULL
#define PCIE1_CONTAINMENT_ENDPOINT_SHIFT      48U
#define PCIE1_CONTAINMENT_DMA_MAGIC           0xD189ULL
#define PCIE1_CONTAINMENT_DMA_SHIFT           48U
#define PCIE1_CONTAINMENT_MSI_MAGIC           0xA189ULL
#define PCIE1_CONTAINMENT_MSI_SHIFT           48U

_Static_assert(PCIE1_CONTAINMENT_DMA_SLOT_COUNT *
               PCIE1_CONTAINMENT_DMA_SLOT_BYTES == PIOS_DMA_PCIE1_SIZE,
               "PCIe1 containment slots must consume the exact DMA arena");
_Static_assert(PCIE1_CONTAINMENT_DMA_PAYLOAD_BYTES > 0U,
               "PCIe1 DMA slots must retain guarded payload space");

enum pcie1_containment_endpoint_state {
    PCIE1_CONTAINMENT_ENDPOINT_FREE = 0U,
    PCIE1_CONTAINMENT_ENDPOINT_ATTACHED,
    PCIE1_CONTAINMENT_ENDPOINT_QUARANTINED,
    PCIE1_CONTAINMENT_ENDPOINT_REMOVED,
};

enum pcie1_containment_dma_state {
    PCIE1_CONTAINMENT_DMA_FREE = 0U,
    PCIE1_CONTAINMENT_DMA_CPU_OWNED,
    PCIE1_CONTAINMENT_DMA_DEVICE_OWNED,
    PCIE1_CONTAINMENT_DMA_CPU_COMPLETE,
    PCIE1_CONTAINMENT_DMA_FAILED,
    PCIE1_CONTAINMENT_DMA_RELEASED,
    PCIE1_CONTAINMENT_DMA_RETIRED,
};

enum pcie1_containment_dma_direction {
    PCIE1_CONTAINMENT_TO_DEVICE = 1U,
    PCIE1_CONTAINMENT_FROM_DEVICE = 2U,
};

enum pcie1_containment_cache_policy {
    PCIE1_CONTAINMENT_CACHE_NORMAL_NC = 1U,
};

enum pcie1_containment_msi_state {
    PCIE1_CONTAINMENT_MSI_UNBOUND = 0U,
    PCIE1_CONTAINMENT_MSI_MASKED,
    PCIE1_CONTAINMENT_MSI_ARMED,
    PCIE1_CONTAINMENT_MSI_EVENT_PENDING,
    PCIE1_CONTAINMENT_MSI_QUEUED,
    PCIE1_CONTAINMENT_MSI_DISPATCHED,
    PCIE1_CONTAINMENT_MSI_QUARANTINED,
};

enum pcie1_containment_fault {
    PCIE1_CONTAINMENT_FAULT_NONE = 0U,
    PCIE1_CONTAINMENT_FAULT_COMPLETION,
    PCIE1_CONTAINMENT_FAULT_RED_ZONE,
    PCIE1_CONTAINMENT_FAULT_TIMEOUT,
    PCIE1_CONTAINMENT_FAULT_AER,
    PCIE1_CONTAINMENT_FAULT_REMOVED,
    PCIE1_CONTAINMENT_FAULT_MSI_PROTOCOL,
    PCIE1_CONTAINMENT_FAULT_MSI_STORM,
    PCIE1_CONTAINMENT_FAULT_GENERATION,
};

struct pcie1_containment_endpoint_handle {
    u64 token;
    u64 generation;
    u32 contract_id;
    u32 bdf;
    u32 _reserved[2];
};

struct pcie1_containment_dma_handle {
    u64 token;
    u64 generation;
    u64 endpoint_generation;
    u64 request_id;
    u32 contract_id;
    u32 _reserved;
};

struct pcie1_containment_msi_ticket {
    u64 token;
    u64 generation;
    u64 endpoint_generation;
    u64 event_seq;
    u32 contract_id;
    u32 intid;
};

struct pcie1_containment_completion {
    u64 endpoint_generation;
    u64 slot_generation;
    u64 request_id;
    u32 actual;
    u32 success;
};

struct pcie1_containment_owner {
    u64 endpoint_generation;
    u64 last_request_id;
    u32 contract_id;
    u32 owner_core;
    u32 state;
    u32 bdf;
    u32 fault;
    u32 fault_count;
    u32 quarantine_count;
    u8 _pad[20U];
} ALIGNED(64);

struct pcie1_containment_msi_control {
    u64 generation;
    u64 event_seq;
    u64 deadline_ms;
    u32 state;
    u32 intid;
    u32 airq_source;
    u32 target_core;
    u32 cause;
    u32 posted_count;
    u32 deferred_count;
    u32 storm_count;
    u32 exhausted;
    u8 _pad[4U];
} ALIGNED(64);

struct pcie1_containment_dma_control {
    u64 generation;
    u64 deadline_ms;
    u64 request_id;
    u32 state;
    u32 actual;
    u32 exhausted;
    u8 _pad[28U];
} ALIGNED(64);

/*
 * Immutable while a DMA handle is live. Numeric addresses are authority for
 * the external endpoint only; neither address is handed across as a raw CPU
 * pointer. The payload is surrounded by one 64-byte red zone on each side.
 */
struct pcie1_containment_dma_span {
    u64 payload_cpu_phys;
    u64 payload_iova;
    u64 endpoint_generation;
    u64 slot_generation;
    u64 request_id;
    u32 used;
    u32 requested;
    u32 capacity;
    u32 slot;
    u8 direction;
    u8 cache_policy;
    u8 guard_lines;
    u8 _reserved;
    u32 _pad;
} ALIGNED(64);

struct pcie1_containment {
    struct pcie1_containment_owner owner;
    struct pcie1_containment_msi_control msi;
    struct pcie1_containment_dma_control
        dma[PCIE1_CONTAINMENT_DMA_SLOT_COUNT];
    struct pcie1_containment_dma_span
        spans[PCIE1_CONTAINMENT_DMA_SLOT_COUNT];
} ALIGNED(64);

struct pcie1_containment_status {
    u64 endpoint_generation;
    u64 msi_event_seq;
    u64 msi_deadline_ms;
    u32 endpoint_state;
    u32 bdf;
    u32 fault;
    u32 fault_count;
    u32 quarantine_count;
    u32 msi_state;
    u32 msi_posted;
    u32 msi_deferred;
    u32 msi_storms;
    u32 active_dma;
    u32 device_owned_dma;
};

_Static_assert(sizeof(struct pcie1_containment_owner) == 64U,
               "PCIe1 endpoint owner must own one cache line");
_Static_assert(sizeof(struct pcie1_containment_msi_control) == 64U,
               "PCIe1 MSI control must own one cache line");
_Static_assert(sizeof(struct pcie1_containment_dma_control) == 64U,
               "PCIe1 DMA controls must own one cache line each");
_Static_assert(sizeof(struct pcie1_containment_dma_span) == 64U,
               "PCIe1 DMA spans must have cache-line stride");
_Static_assert(sizeof(struct pcie1_containment_endpoint_handle) == 32U,
               "PCIe1 endpoint handles must carry full authority");
_Static_assert(sizeof(struct pcie1_containment_dma_handle) == 40U,
               "PCIe1 DMA handles must carry full authority");
_Static_assert(sizeof(struct pcie1_containment_msi_ticket) == 40U,
               "PCIe1 MSI tickets must carry sequence authority");
_Static_assert(sizeof(struct pcie1_containment_completion) == 32U,
               "PCIe1 completion identity must have fixed wire size");
_Static_assert(__builtin_offsetof(struct pcie1_containment, msi) == 64U,
               "PCIe1 MSI state must not share the endpoint cache line");
_Static_assert(__builtin_offsetof(struct pcie1_containment, dma) == 128U,
               "PCIe1 DMA controls require a dedicated cache-line region");
_Static_assert(__builtin_offsetof(struct pcie1_containment, spans) ==
               128U + PCIE1_CONTAINMENT_DMA_SLOT_COUNT * 64U,
               "PCIe1 DMA spans must not share mutable control lines");

static inline u32 pcie1_containment_bdf(u32 bus, u32 device, u32 function)
{
    if (bus == 0U || bus > 255U || device > 31U || function > 7U)
        return 0U;
    return (bus << 8) | (device << 3) | function;
}

bool pcie1_containment_bdf_valid(u32 bdf);

/*
 * `endpoint_generation` must come from the future #188 endpoint lease.
 * Initialization is one-shot over fresh zeroed storage and creates no hardware
 * authority. `hardware_enable_allowed()` remains false until live proof lands.
 */
bool pcie1_containment_init(
    struct pcie1_containment *contract, u32 contract_id, u32 bdf,
    u64 endpoint_generation, u32 caller_core,
    struct pcie1_containment_endpoint_handle *endpoint_out);

bool pcie1_containment_dma_acquire(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u64 request_id, u32 requested,
    enum pcie1_containment_dma_direction direction,
    u64 now_ms, u64 timeout_ms,
    struct pcie1_containment_dma_handle *handle_out);
bool pcie1_containment_dma_publish(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core);
bool pcie1_containment_completion_submit(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_msi_ticket *ticket,
    const struct pcie1_containment_completion *completion, u32 caller_core,
    bool guards_intact);
bool pcie1_containment_dma_cancel(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core);
bool pcie1_containment_dma_expire(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle,
    u32 caller_core, u64 now_ms);
bool pcie1_containment_dma_release(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core);

bool pcie1_containment_msi_bind(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u32 intid, u32 airq_source, u32 target_core,
    bool line_masked, bool airq_handler_registered);
bool pcie1_containment_msi_arm(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core);
bool pcie1_containment_msi_top_half(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u32 intid, u32 cause, bool acknowledged,
    bool line_masked, u64 now_ms, u64 timeout_ms,
    struct pcie1_containment_msi_ticket *ticket_out);
bool pcie1_containment_msi_publish_result(
    struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket,
    u32 caller_core, bool airq_posted);
/* Called only by the scheduled AIRQ handler. Performs the system acquire
 * barrier before any completion record, payload, or red zone is inspected. */
bool pcie1_containment_msi_dispatch_begin(
    struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket, u32 caller_core);
bool pcie1_containment_msi_pending_ticket(
    const struct pcie1_containment *contract, u32 caller_core,
    struct pcie1_containment_msi_ticket *ticket_out);
bool pcie1_containment_msi_dispatch_complete(
    struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket, u32 caller_core);
bool pcie1_containment_msi_expire(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u64 now_ms);

bool pcie1_containment_quarantine(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, enum pcie1_containment_fault fault);
bool pcie1_containment_remove(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core);

bool pcie1_containment_dma_state_get(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    enum pcie1_containment_dma_state *state_out);
bool pcie1_containment_dma_span_get(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    struct pcie1_containment_dma_span *span_out);
bool pcie1_containment_dma_completion_get(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    u32 *actual_out, bool *success_out);
bool pcie1_containment_status_get(
    const struct pcie1_containment *contract, u32 caller_core,
    struct pcie1_containment_status *status_out);

/* Deliberately false until #188 and the live #189 brick test are proven. */
bool pcie1_containment_hardware_enable_allowed(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core);
