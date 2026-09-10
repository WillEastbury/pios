/*
 * dwc2_contract.c - pure DWC2 DMA-span and transfer-lifecycle contract.
 */
#include "types.h"
#include "dwc2_contract.h"

static void dwc2_clear_handle(struct dwc2_transfer_handle *handle)
{
    if (!handle)
        return;
    handle->generation = 0U;
    handle->slot = 0U;
    handle->_reserved = 0U;
}

static bool dwc2_handle_record(struct dwc2_transfer_pool *pool,
                               const struct dwc2_transfer_handle *handle,
                               struct dwc2_transfer_record **record_out)
{
    struct dwc2_transfer_record *record;

    if (!pool || !handle || !record_out ||
        handle->slot >= DWC2_TRANSFER_CAPACITY || handle->generation == 0U)
        return false;
    record = &pool->records[handle->slot];
    if (record->generation != handle->generation ||
        record->state == DWC2_TRANSFER_FREE)
        return false;
    *record_out = record;
    return true;
}

static bool dwc2_handle_record_const(
    const struct dwc2_transfer_pool *pool,
    const struct dwc2_transfer_handle *handle,
    const struct dwc2_transfer_record **record_out)
{
    const struct dwc2_transfer_record *record;

    if (!pool || !handle || !record_out ||
        handle->slot >= DWC2_TRANSFER_CAPACITY || handle->generation == 0U)
        return false;
    record = &pool->records[handle->slot];
    if (record->generation != handle->generation ||
        record->state == DWC2_TRANSFER_FREE)
        return false;
    *record_out = record;
    return true;
}

void dwc2_transfer_pool_init(struct dwc2_transfer_pool *pool)
{
    u32 i;

    if (!pool)
        return;
    for (i = 0U; i < DWC2_TRANSFER_CAPACITY; i++) {
        struct dwc2_transfer_record *record = &pool->records[i];

        record->generation = (u64)i + 1U;
        record->cpu_phys = 0U;
        record->dma_bus = 0U;
        record->capacity = 0U;
        record->requested = 0U;
        record->actual = 0U;
        record->state = DWC2_TRANSFER_FREE;
    }
}

bool dwc2_dma_bus_addr(u64 cpu_phys, u32 length, u32 capacity,
                       u64 *bus_addr_out)
{
    u64 end;

    if (!bus_addr_out)
        return false;
    *bus_addr_out = 0U;
    if (cpu_phys == 0U || length == 0U || capacity == 0U ||
        length > capacity ||
        (cpu_phys & (DWC2_DMA_ALIGNMENT - 1U)) != 0U ||
        (length & (DWC2_DMA_ALIGNMENT - 1U)) != 0U ||
        (capacity & (DWC2_DMA_ALIGNMENT - 1U)) != 0U)
        return false;
    end = cpu_phys + (u64)capacity;
    if (end < cpu_phys || cpu_phys >= DWC2_DMA_PHYS_LIMIT ||
        end > DWC2_DMA_PHYS_LIMIT)
        return false;

    *bus_addr_out = DWC2_DMA_BUS_ALIAS | cpu_phys;
    return true;
}

bool dwc2_transfer_prepare(struct dwc2_transfer_pool *pool, u64 cpu_phys,
                           u32 length, u32 capacity,
                           struct dwc2_transfer_handle *handle_out)
{
    u64 dma_bus;
    u32 i;

    dwc2_clear_handle(handle_out);
    if (!pool || !handle_out ||
        !dwc2_dma_bus_addr(cpu_phys, length, capacity, &dma_bus))
        return false;
    for (i = 0U; i < DWC2_TRANSFER_CAPACITY; i++) {
        struct dwc2_transfer_record *record = &pool->records[i];

        if (record->state != DWC2_TRANSFER_FREE)
            continue;
        record->cpu_phys = cpu_phys;
        record->dma_bus = dma_bus;
        record->capacity = capacity;
        record->requested = length;
        record->actual = 0U;
        record->state = DWC2_TRANSFER_PREPARED;
        handle_out->generation = record->generation;
        handle_out->slot = i;
        handle_out->_reserved = 0U;
        return true;
    }
    return false;
}

bool dwc2_transfer_submit(struct dwc2_transfer_pool *pool,
                          const struct dwc2_transfer_handle *handle)
{
    struct dwc2_transfer_record *record;

    if (!dwc2_handle_record(pool, handle, &record) ||
        record->state != DWC2_TRANSFER_PREPARED)
        return false;
    record->state = DWC2_TRANSFER_HARDWARE;
    return true;
}

bool dwc2_transfer_complete(struct dwc2_transfer_pool *pool,
                            const struct dwc2_transfer_handle *handle,
                            u32 actual)
{
    struct dwc2_transfer_record *record;

    if (!dwc2_handle_record(pool, handle, &record) ||
        record->state != DWC2_TRANSFER_HARDWARE || actual > record->requested)
        return false;
    record->actual = actual;
    record->state = DWC2_TRANSFER_COMPLETED;
    return true;
}

bool dwc2_transfer_fail(struct dwc2_transfer_pool *pool,
                        const struct dwc2_transfer_handle *handle)
{
    struct dwc2_transfer_record *record;

    if (!dwc2_handle_record(pool, handle, &record) ||
        record->state != DWC2_TRANSFER_HARDWARE)
        return false;
    record->state = DWC2_TRANSFER_FAILED;
    return true;
}

bool dwc2_transfer_release(struct dwc2_transfer_pool *pool,
                           const struct dwc2_transfer_handle *handle)
{
    struct dwc2_transfer_record *record;

    if (!dwc2_handle_record(pool, handle, &record) ||
        (record->state != DWC2_TRANSFER_COMPLETED &&
         record->state != DWC2_TRANSFER_FAILED))
        return false;

    record->cpu_phys = 0U;
    record->dma_bus = 0U;
    record->capacity = 0U;
    record->requested = 0U;
    record->actual = 0U;
    record->generation++;
    if (record->generation == 0U)
        record->generation = 1U;
    record->state = DWC2_TRANSFER_FREE;
    return true;
}

bool dwc2_transfer_get_state(const struct dwc2_transfer_pool *pool,
                             const struct dwc2_transfer_handle *handle,
                             enum dwc2_transfer_state *state_out)
{
    const struct dwc2_transfer_record *record;

    if (!state_out || !dwc2_handle_record_const(pool, handle, &record))
        return false;
    *state_out = (enum dwc2_transfer_state)record->state;
    return true;
}

bool dwc2_transfer_get_dma_bus(const struct dwc2_transfer_pool *pool,
                               const struct dwc2_transfer_handle *handle,
                               u64 *bus_addr_out)
{
    const struct dwc2_transfer_record *record;

    if (!bus_addr_out)
        return false;
    *bus_addr_out = 0U;
    if (!dwc2_handle_record_const(pool, handle, &record) ||
        (record->state != DWC2_TRANSFER_PREPARED &&
         record->state != DWC2_TRANSFER_HARDWARE))
        return false;
    *bus_addr_out = record->dma_bus;
    return true;
}
