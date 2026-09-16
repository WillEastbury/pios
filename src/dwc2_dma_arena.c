/*
 * dwc2_dma_arena.c - pure BCM2837 DWC2 DMA-arena ownership contract.
 *
 * The reserved arena is Normal-NC in every CPU alias. Publication and
 * consumption therefore use system-scoped barriers only: no dcache operation
 * belongs here. Inner-shareable barriers alone do not order an external DMA
 * master.
 * This module neither authorizes a BCM bus alias as a CPU mapping nor starts
 * a hardware transfer.
 */
#include "types.h"
#include "dwc2_dma_arena.h"

static void dwc2_dma_arena_clear_handle(struct dwc2_dma_arena_handle *handle)
{
    if (!handle)
        return;
    handle->token = 0U;
    handle->generation = 0U;
    handle->controller_id = 0U;
    handle->_reserved = 0U;
}

static bool dwc2_dma_arena_fresh(const struct dwc2_dma_arena *arena)
{
    const u8 *bytes = (const u8 *)arena;
    usize i;

    if (!arena)
        return false;
    for (i = 0U; i < sizeof(*arena); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
}

static bool dwc2_dma_arena_layout_valid(void)
{
#if PIOS_PLATFORM == PIOS_PLATFORM_PI3 || \
    PIOS_PLATFORM == PIOS_PLATFORM_PIZERO2W
    return PIOS_DWC2_DMA_BASE == 0x06400000UL &&
           PIOS_DWC2_DMA_SIZE == 0x00200000UL &&
           PIOS_DWC2_DMA_BASE + PIOS_DWC2_DMA_SIZE == 0x06600000UL;
#else
    return false;
#endif
}

static u64 dwc2_dma_arena_token(u32 slot)
{
    return (DWC2_DMA_ARENA_TOKEN_MAGIC << DWC2_DMA_ARENA_TOKEN_SHIFT) |
           (u64)slot;
}

static bool dwc2_dma_arena_direction_valid(enum dwc2_dma_direction direction)
{
    return direction == DWC2_DMA_TO_DEVICE ||
           direction == DWC2_DMA_FROM_DEVICE;
}

static void dwc2_dma_arena_publish_state(
    struct dwc2_dma_arena_slot_control *control, u32 state)
{
    dmb_ishst();
    control->state = state;
    dmb_ishst();
}

static void dwc2_dma_arena_span_clear(struct dwc2_dma_arena_span *span)
{
    u8 *bytes = (u8 *)span;
    usize i;

    for (i = 0U; i < sizeof(*span); i++)
        bytes[i] = 0U;
}

static bool dwc2_dma_arena_owner_valid(const struct dwc2_dma_arena *arena,
                                       u32 controller_id, u32 caller_core)
{
    return arena && caller_core == DWC2_DMA_ARENA_OWNER_CORE &&
           controller_id != 0U && arena->owner.initialized == 1U &&
           arena->owner.owner_core == DWC2_DMA_ARENA_OWNER_CORE &&
           arena->owner.controller_id == controller_id;
}

static bool dwc2_dma_arena_handle_control(
    struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle,
    struct dwc2_dma_arena_slot_control **control_out, u32 *slot_out)
{
    u32 slot;

    if (!arena || !handle || !control_out ||
        !dwc2_dma_arena_owner_valid(arena, handle->controller_id, caller_core) ||
        handle->_reserved != 0U || handle->generation == 0U ||
        (handle->token & ~DWC2_DMA_ARENA_TOKEN_MASK) != 0U ||
        (handle->token >> DWC2_DMA_ARENA_TOKEN_SHIFT) !=
        DWC2_DMA_ARENA_TOKEN_MAGIC)
        return false;
    slot = (u32)(handle->token & DWC2_DMA_ARENA_TOKEN_SLOT_MASK);
    if (slot >= DWC2_DMA_ARENA_SLOT_COUNT)
        return false;
    dmb_ishld();
    if (
        arena->controls[slot].generation != handle->generation ||
        arena->controls[slot].state == DWC2_DMA_ARENA_FREE ||
        arena->controls[slot].state == DWC2_DMA_ARENA_RELEASED)
        return false;
    *control_out = &arena->controls[slot];
    if (slot_out)
        *slot_out = slot;
    return true;
}

static bool dwc2_dma_arena_handle_control_const(
    const struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle,
    const struct dwc2_dma_arena_slot_control **control_out, u32 *slot_out)
{
    const struct dwc2_dma_arena_slot_control *control;
    u32 slot;

    if (!arena || !handle || !control_out ||
        !dwc2_dma_arena_owner_valid(arena, handle->controller_id, caller_core) ||
        handle->_reserved != 0U || handle->generation == 0U ||
        (handle->token & ~DWC2_DMA_ARENA_TOKEN_MASK) != 0U ||
        (handle->token >> DWC2_DMA_ARENA_TOKEN_SHIFT) !=
        DWC2_DMA_ARENA_TOKEN_MAGIC)
        return false;
    slot = (u32)(handle->token & DWC2_DMA_ARENA_TOKEN_SLOT_MASK);
    if (slot >= DWC2_DMA_ARENA_SLOT_COUNT)
        return false;
    control = &arena->controls[slot];
    dmb_ishld();
    if (control->generation != handle->generation ||
        control->state == DWC2_DMA_ARENA_FREE ||
        control->state == DWC2_DMA_ARENA_RELEASED)
        return false;
    *control_out = control;
    if (slot_out)
        *slot_out = slot;
    return true;
}

bool dwc2_dma_arena_init(struct dwc2_dma_arena *arena, u32 controller_id,
                         u32 caller_core)
{
    u32 i;

    if (!arena || controller_id == 0U ||
        caller_core != DWC2_DMA_ARENA_OWNER_CORE ||
        !dwc2_dma_arena_layout_valid() || !dwc2_dma_arena_fresh(arena))
        return false;
    arena->owner.controller_id = controller_id;
    arena->owner.owner_core = DWC2_DMA_ARENA_OWNER_CORE;
    arena->owner.initialized = 1U;
    for (i = 0U; i < DWC2_DMA_ARENA_SLOT_COUNT; i++) {
        arena->controls[i].generation = (u64)i + 1U;
        arena->controls[i].state = DWC2_DMA_ARENA_FREE;
        arena->controls[i].actual = 0U;
        arena->controls[i].exhausted = 0U;
    }
    dmb_ishst();
    return true;
}

bool dwc2_dma_arena_acquire(struct dwc2_dma_arena *arena, u32 controller_id,
                            u32 caller_core, u32 requested,
                            enum dwc2_dma_direction direction,
                            struct dwc2_dma_arena_handle *handle_out)
{
    u32 i;

    dwc2_dma_arena_clear_handle(handle_out);
    if (!handle_out || !dwc2_dma_arena_owner_valid(arena, controller_id,
                                                    caller_core) ||
        requested == 0U || requested > DWC2_DMA_ARENA_SLOT_SIZE ||
        (requested & (DWC2_DMA_ALIGNMENT - 1U)) != 0U ||
        !dwc2_dma_arena_direction_valid(direction))
        return false;
    for (i = 0U; i < DWC2_DMA_ARENA_SLOT_COUNT; i++) {
        struct dwc2_dma_arena_slot_control *control = &arena->controls[i];
        struct dwc2_dma_arena_span *span = &arena->spans[i];
        u64 cpu_phys;
        u64 bcm_bus_addr;

        if (control->state != DWC2_DMA_ARENA_FREE || control->exhausted != 0U)
            continue;
        cpu_phys = PIOS_DWC2_DMA_BASE + (u64)i * DWC2_DMA_ARENA_SLOT_SIZE;
        if (!dwc2_dma_bus_addr(cpu_phys, requested, DWC2_DMA_ARENA_SLOT_SIZE,
                               &bcm_bus_addr))
            return false;
        span->cpu_phys = cpu_phys;
        span->bcm_bus_addr = bcm_bus_addr;
        span->generation = control->generation;
        span->used = requested;
        span->requested = requested;
        span->capacity = DWC2_DMA_ARENA_SLOT_SIZE;
        span->slot = i;
        span->direction = (u32)direction;
        span->cache_policy = DWC2_DMA_CACHE_NORMAL_NC;
        span->_reserved = 0U;
        control->actual = 0U;
        dwc2_dma_arena_publish_state(control, DWC2_DMA_ARENA_CPU_OWNED);
        handle_out->token = dwc2_dma_arena_token(i);
        handle_out->generation = control->generation;
        handle_out->controller_id = controller_id;
        handle_out->_reserved = 0U;
        return true;
    }
    return false;
}

bool dwc2_dma_arena_publish_to_device(
    struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle)
{
    struct dwc2_dma_arena_slot_control *control;

    if (!dwc2_dma_arena_handle_control(arena, caller_core, handle, &control,
                                       NULL) ||
        control->state != DWC2_DMA_ARENA_CPU_OWNED)
        return false;
    /* Order payload writes before ownership reaches the external DMA master. */
    dmb();
    dwc2_dma_arena_publish_state(control, DWC2_DMA_ARENA_DEVICE_OWNED);
    return true;
}

bool dwc2_dma_arena_complete_from_device(
    struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle, u32 actual, bool success)
{
    struct dwc2_dma_arena_slot_control *control;
    u32 slot;

    if (!dwc2_dma_arena_handle_control(arena, caller_core, handle, &control,
                                       &slot) ||
        control->state != DWC2_DMA_ARENA_DEVICE_OWNED ||
        actual > arena->spans[slot].requested)
        return false;
    /* Order external DMA writes before CPU ownership and consumption. */
    dmb();
    control->actual = actual;
    dwc2_dma_arena_publish_state(control, success ? DWC2_DMA_ARENA_CPU_COMPLETE :
                                  DWC2_DMA_ARENA_FAILED);
    return true;
}

bool dwc2_dma_arena_cancel(struct dwc2_dma_arena *arena, u32 caller_core,
                           const struct dwc2_dma_arena_handle *handle)
{
    struct dwc2_dma_arena_slot_control *control;

    if (!dwc2_dma_arena_handle_control(arena, caller_core, handle, &control,
                                       NULL) ||
        control->state != DWC2_DMA_ARENA_CPU_OWNED)
        return false;
    dwc2_dma_arena_publish_state(control, DWC2_DMA_ARENA_FAILED);
    return true;
}

bool dwc2_dma_arena_release(struct dwc2_dma_arena *arena, u32 caller_core,
                            const struct dwc2_dma_arena_handle *handle)
{
    struct dwc2_dma_arena_slot_control *control;
    u32 slot;

    if (!dwc2_dma_arena_handle_control(arena, caller_core, handle, &control,
                                       &slot) ||
        (control->state != DWC2_DMA_ARENA_CPU_OWNED &&
         control->state != DWC2_DMA_ARENA_CPU_COMPLETE &&
         control->state != DWC2_DMA_ARENA_FAILED))
        return false;
    dwc2_dma_arena_publish_state(control, DWC2_DMA_ARENA_RELEASED);
    /* Invalidate every old handle before wiping the immutable span. */
    if (control->generation == ~0ULL)
        control->exhausted = 1U;
    else
        control->generation++;
    dmb_ishst();
    dwc2_dma_arena_span_clear(&arena->spans[slot]);
    control->actual = 0U;
    if (control->exhausted == 0U)
        dwc2_dma_arena_publish_state(control, DWC2_DMA_ARENA_FREE);
    return true;
}

bool dwc2_dma_arena_state_get(const struct dwc2_dma_arena *arena,
                              u32 caller_core,
                              const struct dwc2_dma_arena_handle *handle,
                              enum dwc2_dma_arena_state *state_out)
{
    const struct dwc2_dma_arena_slot_control *control;

    if (!state_out ||
        !dwc2_dma_arena_handle_control_const(arena, caller_core, handle,
                                              &control, NULL))
        return false;
    dmb_ishld();
    *state_out = (enum dwc2_dma_arena_state)control->state;
    return true;
}

bool dwc2_dma_arena_span_get(const struct dwc2_dma_arena *arena,
                             u32 caller_core,
                             const struct dwc2_dma_arena_handle *handle,
                             struct dwc2_dma_arena_span *span_out)
{
    const struct dwc2_dma_arena_slot_control *control;
    u32 slot;

    if (!span_out ||
        !dwc2_dma_arena_handle_control_const(arena, caller_core, handle,
                                              &control, &slot))
        return false;
    dmb_ishld();
    *span_out = arena->spans[slot];
    return true;
}

bool dwc2_dma_arena_completion_get(
    const struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle, u32 *actual_out,
    bool *success_out)
{
    const struct dwc2_dma_arena_slot_control *control;

    if (actual_out)
        *actual_out = 0U;
    if (success_out)
        *success_out = false;
    if (!actual_out || !success_out ||
        !dwc2_dma_arena_handle_control_const(arena, caller_core, handle,
                                              &control, NULL) ||
        (control->state != DWC2_DMA_ARENA_CPU_COMPLETE &&
         control->state != DWC2_DMA_ARENA_FAILED))
        return false;
    dmb_ishld();
    *actual_out = control->actual;
    *success_out = control->state == DWC2_DMA_ARENA_CPU_COMPLETE;
    return true;
}

bool dwc2_dma_arena_cache_contract_get(
    const struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle,
    enum dwc2_dma_cache_policy *policy_out, bool *publish_barrier_only_out,
    bool *consume_barrier_only_out,
    enum dwc2_dma_barrier_scope *barrier_scope_out)
{
    struct dwc2_dma_arena_span span;

    if (policy_out)
        *policy_out = 0U;
    if (publish_barrier_only_out)
        *publish_barrier_only_out = false;
    if (consume_barrier_only_out)
        *consume_barrier_only_out = false;
    if (barrier_scope_out)
        *barrier_scope_out = 0U;
    if (!policy_out || !publish_barrier_only_out || !consume_barrier_only_out ||
        !barrier_scope_out ||
        !dwc2_dma_arena_span_get(arena, caller_core, handle, &span))
        return false;
    *policy_out = (enum dwc2_dma_cache_policy)span.cache_policy;
    *publish_barrier_only_out = true;
    *consume_barrier_only_out = true;
    *barrier_scope_out = DWC2_DMA_BARRIER_SYSTEM;
    return true;
}
