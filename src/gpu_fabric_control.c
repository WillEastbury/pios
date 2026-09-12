/*
 * gpu_fabric_control.c - ADR-070 bounded offline GPU-fabric control plane.
 *
 * There is deliberately no execution adapter here. All loops have fixed
 * maxima, mutable records have dedicated cache lines, and Core 0 masks local
 * IRQs over a transition so an IRQ-side transition cannot re-enter it.
 */
#include "types.h"
#include "gpu_fabric_control.h"

static u64 gpu_fabric_irq_save(void)
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

static void gpu_fabric_irq_restore(u64 daif)
{
#ifndef PIOS_HOST_TYPES_SHIM
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#else
    (void)daif;
#endif
}

static void gpu_fabric_zero(void *value, usize bytes)
{
    u8 *p = (u8 *)value;
    usize i;

    for (i = 0U; i < bytes; i++)
        p[i] = 0U;
}

static bool gpu_fabric_fresh(const struct gpu_fabric *fabric)
{
    const u8 *bytes = (const u8 *)fabric;
    usize i;

    if (!fabric)
        return false;
    for (i = 0U; i < sizeof(*fabric); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
}

static void gpu_fabric_clear_node_handle(struct gpu_fabric_node_handle *out)
{
    if (out)
        gpu_fabric_zero(out, sizeof(*out));
}

static void gpu_fabric_clear_placement_handle(
    struct gpu_fabric_placement_handle *out)
{
    if (out)
        gpu_fabric_zero(out, sizeof(*out));
}

static void gpu_fabric_clear_activation_handle(
    struct gpu_fabric_activation_handle *out)
{
    if (out)
        gpu_fabric_zero(out, sizeof(*out));
}

static bool gpu_fabric_ready(const struct gpu_fabric *fabric, u32 caller_core)
{
    return fabric && caller_core == GPU_FABRIC_OWNER_CORE &&
           core_id() == GPU_FABRIC_OWNER_CORE &&
           fabric->owner.initialized == 1U &&
           fabric->owner.controller_id != 0U &&
           fabric->owner.hardware_enable == 0U;
}

static bool gpu_fabric_begin(struct gpu_fabric *fabric, u32 caller_core,
                             u64 *daif_out)
{
    u64 daif;

    if (!daif_out || !gpu_fabric_ready(fabric, caller_core))
        return false;
    daif = gpu_fabric_irq_save();
    if (fabric->owner.transition_active != 0U) {
        gpu_fabric_irq_restore(daif);
        return false;
    }
    fabric->owner.transition_active = 1U;
    dmb_ishst();
    *daif_out = daif;
    return true;
}

static void gpu_fabric_end(struct gpu_fabric *fabric, u64 daif)
{
    dmb_ishst();
    fabric->owner.transition_active = 0U;
    gpu_fabric_irq_restore(daif);
}

static bool gpu_fabric_generation_bump(u32 *generation)
{
    if (!generation || *generation == ~0U)
        return false;
    (*generation)++;
    return true;
}

static bool gpu_fabric_health_valid(enum gpu_fabric_health health)
{
    return health >= GPU_FABRIC_HEALTH_UNKNOWN &&
           health <= GPU_FABRIC_HEALTH_FAILED;
}

static bool gpu_fabric_node_facts_valid(
    const struct gpu_fabric_node_facts *facts)
{
    return facts && facts->node_id != 0U && facts->_reserved == 0U &&
           facts->epoch != 0U &&
           facts->supported_kernel_mask != 0U &&
           facts->verified_vram_bytes != 0U &&
           facts->resident_capacity_bytes != 0U &&
           facts->resident_capacity_bytes <= facts->verified_vram_bytes &&
           facts->resident_used_bytes <= facts->resident_capacity_bytes &&
           gpu_fabric_health_valid((enum gpu_fabric_health)facts->health) &&
           facts->load_per_mille <= 1000U;
}

static u32 gpu_fabric_node_slot(const struct gpu_fabric *fabric, u32 node_id)
{
    u32 i;

    for (i = 0U; i < GPU_FABRIC_MAX_NODES; i++) {
        if (fabric->nodes[i].state != GPU_FABRIC_NODE_EMPTY &&
            fabric->nodes[i].node_id == node_id)
            return i;
    }
    return GPU_FABRIC_MAX_NODES;
}

static u32 gpu_fabric_node_slot_generation(const struct gpu_fabric *fabric,
                                           u32 node_id, u32 generation)
{
    u32 slot = gpu_fabric_node_slot(fabric, node_id);

    if (slot == GPU_FABRIC_MAX_NODES ||
        fabric->nodes[slot].generation != generation)
        return GPU_FABRIC_MAX_NODES;
    return slot;
}

static bool gpu_fabric_node_handle_valid(const struct gpu_fabric *fabric,
                                         const struct gpu_fabric_node_handle *h,
                                         u32 *slot_out)
{
    u32 slot;

    if (!fabric || !h || h->controller_id != fabric->owner.controller_id ||
        h->node_id == 0U || h->generation == 0U || h->_reserved != 0U)
        return false;
    slot = gpu_fabric_node_slot_generation(fabric, h->node_id, h->generation);
    if (slot == GPU_FABRIC_MAX_NODES || fabric->nodes[slot].epoch != h->epoch ||
        fabric->nodes[slot].state == GPU_FABRIC_NODE_EMPTY ||
        fabric->nodes[slot].state == GPU_FABRIC_NODE_REMOVED)
        return false;
    if (slot_out)
        *slot_out = slot;
    return true;
}

static bool gpu_fabric_node_eligible(const struct gpu_fabric_node_record *node,
                                     u64 required_kernel_mask)
{
    return node && node->state == GPU_FABRIC_NODE_ELIGIBLE &&
           node->health == GPU_FABRIC_HEALTH_OK &&
           node->gpu_verified == 1U &&
           (node->supported_kernel_mask & required_kernel_mask) ==
               required_kernel_mask &&
           node->resident_used_bytes <= node->resident_capacity_bytes &&
           node->reserved_bytes <=
               node->resident_capacity_bytes - node->resident_used_bytes;
}

static u64 gpu_fabric_node_available(const struct gpu_fabric_node_record *node)
{
    return node->resident_capacity_bytes - node->resident_used_bytes -
           node->reserved_bytes;
}

static void gpu_fabric_activation_invalidate_slot(struct gpu_fabric *fabric,
                                                  u32 slot,
                                                  enum gpu_fabric_activation_state state)
{
    if (fabric->activations[slot].state == GPU_FABRIC_ACTIVATION_EMPTY)
        return;
    if (gpu_fabric_generation_bump(&fabric->activations[slot].generation))
        fabric->activations[slot].state = (u32)state;
    else
        fabric->activations[slot].state = GPU_FABRIC_ACTIVATION_RETIRED;
}

static void gpu_fabric_release_placement_reservations(
    struct gpu_fabric *fabric, u32 placement_slot)
{
    struct gpu_fabric_placement_record *placement =
        &fabric->placements[placement_slot];
    u32 i;

    for (i = 0U; i < placement->shard_count; i++) {
        struct gpu_fabric_shard_record *shard =
            &fabric->shards[placement_slot][i];
        u32 node_slot = gpu_fabric_node_slot(fabric, shard->node_id);

        if (node_slot != GPU_FABRIC_MAX_NODES &&
            fabric->nodes[node_slot].reserved_bytes >= shard->bytes)
            fabric->nodes[node_slot].reserved_bytes -= shard->bytes;
    }
}

static void gpu_fabric_invalidate_placement(struct gpu_fabric *fabric,
                                            u32 placement_slot,
                                            enum gpu_fabric_placement_state state)
{
    struct gpu_fabric_placement_record *placement =
        &fabric->placements[placement_slot];
    u32 i;

    if (placement->state != GPU_FABRIC_PLACEMENT_ACTIVE)
        return;
    gpu_fabric_release_placement_reservations(fabric, placement_slot);
    placement->state = (u32)state;
    for (i = 0U; i < GPU_FABRIC_MAX_ACTIVATIONS; i++) {
        if (fabric->activations[i].state != GPU_FABRIC_ACTIVATION_EMPTY &&
            fabric->activations[i].placement_slot == placement_slot &&
            fabric->activations[i].placement_generation ==
                placement->generation)
            gpu_fabric_activation_invalidate_slot(
                fabric, i, GPU_FABRIC_ACTIVATION_INVALID);
    }
}

static void gpu_fabric_invalidate_node_placements(struct gpu_fabric *fabric,
                                                  u32 node_id)
{
    u32 placement_slot;
    u32 shard_slot;

    for (placement_slot = 0U; placement_slot < GPU_FABRIC_MAX_PLACEMENTS;
         placement_slot++) {
        struct gpu_fabric_placement_record *placement =
            &fabric->placements[placement_slot];
        bool used = false;

        if (placement->state != GPU_FABRIC_PLACEMENT_ACTIVE)
            continue;
        for (shard_slot = 0U; shard_slot < placement->shard_count;
             shard_slot++) {
            if (fabric->shards[placement_slot][shard_slot].node_id == node_id) {
                used = true;
                break;
            }
        }
        if (used)
            gpu_fabric_invalidate_placement(
                fabric, placement_slot, GPU_FABRIC_PLACEMENT_INVALID);
    }
}

static bool gpu_fabric_placement_handle_valid(
    const struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_handle *handle, u32 *slot_out)
{
    const struct gpu_fabric_placement_record *placement;

    if (!fabric || !handle || handle->controller_id != fabric->owner.controller_id ||
        handle->slot >= GPU_FABRIC_MAX_PLACEMENTS || handle->generation == 0U ||
        handle->_reserved != 0U)
        return false;
    placement = &fabric->placements[handle->slot];
    if (placement->generation != handle->generation ||
        placement->model_id != handle->model_id ||
        placement->model_generation != handle->model_generation ||
        handle->model_id == 0U || handle->model_generation == 0U)
        return false;
    if (slot_out)
        *slot_out = handle->slot;
    return true;
}

static bool gpu_fabric_placement_active(
    const struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_handle *handle, u32 *slot_out)
{
    u32 slot;

    if (!gpu_fabric_placement_handle_valid(fabric, handle, &slot) ||
        fabric->placements[slot].state != GPU_FABRIC_PLACEMENT_ACTIVE)
        return false;
    if (slot_out)
        *slot_out = slot;
    return true;
}

static u32 gpu_fabric_shard_slot(const struct gpu_fabric *fabric,
                                 u32 placement_slot, u64 shard_id)
{
    u32 i;

    for (i = 0U; i < fabric->placements[placement_slot].shard_count; i++) {
        if (fabric->shards[placement_slot][i].shard_id == shard_id)
            return i;
    }
    return GPU_FABRIC_MAX_SHARDS;
}

static bool gpu_fabric_activation_handle_valid(
    const struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_handle *handle, u32 *slot_out)
{
    u32 slot;

    if (!fabric || !handle || handle->controller_id != fabric->owner.controller_id ||
        handle->slot >= GPU_FABRIC_MAX_ACTIVATIONS || handle->generation == 0U ||
        handle->sequence == 0U || handle->_reserved != 0U)
        return false;
    slot = handle->slot;
    if (fabric->activations[slot].generation != handle->generation ||
        fabric->routes[slot].sequence != handle->sequence)
        return false;
    if (slot_out)
        *slot_out = slot;
    return true;
}

static u32 gpu_fabric_flow_slot(const struct gpu_fabric *fabric,
                                u32 placement_slot, u64 source_shard,
                                u64 target_shard)
{
    u32 i;

    for (i = 0U; i < GPU_FABRIC_MAX_FLOWS; i++) {
        const struct gpu_fabric_flow_record *flow =
            &fabric->flows[placement_slot][i];
        if (flow->in_use != 0U && flow->source_shard_id == source_shard &&
            flow->target_shard_id == target_shard)
            return i;
    }
    return GPU_FABRIC_MAX_FLOWS;
}

bool gpu_fabric_init(struct gpu_fabric *fabric, u32 controller_id,
                     u32 caller_core)
{
    u64 daif;

    if (!fabric || controller_id == 0U ||
        caller_core != GPU_FABRIC_OWNER_CORE ||
        core_id() != GPU_FABRIC_OWNER_CORE ||
        !gpu_fabric_fresh(fabric))
        return false;
    daif = gpu_fabric_irq_save();
    fabric->owner.controller_id = controller_id;
    fabric->owner.initialized = 1U;
    fabric->owner.hardware_enable = 0U;
    fabric->owner.transition_active = 0U;
    dmb_ishst();
    gpu_fabric_irq_restore(daif);
    return true;
}

bool gpu_fabric_node_advertise(struct gpu_fabric *fabric,
                               const struct gpu_fabric_node_facts *facts,
                               bool gpu_verified, u32 caller_core,
                               struct gpu_fabric_node_handle *node_out)
{
    u64 daif;
    u32 slot;
    u32 generation;
    struct gpu_fabric_node_record *node;

    gpu_fabric_clear_node_handle(node_out);
    if (!gpu_fabric_begin(fabric, caller_core, &daif))
        return false;
    if (!node_out || !gpu_fabric_node_facts_valid(facts)) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    slot = gpu_fabric_node_slot(fabric, facts->node_id);
    if (slot != GPU_FABRIC_MAX_NODES) {
        node = &fabric->nodes[slot];
        if (node->state != GPU_FABRIC_NODE_REMOVED ||
            facts->epoch <= node->epoch ||
            !gpu_fabric_generation_bump(&node->generation)) {
            gpu_fabric_end(fabric, daif);
            return false;
        }
        generation = node->generation;
    } else {
        for (slot = 0U; slot < GPU_FABRIC_MAX_NODES; slot++) {
            if (fabric->nodes[slot].state == GPU_FABRIC_NODE_EMPTY ||
                fabric->nodes[slot].state == GPU_FABRIC_NODE_REMOVED)
                break;
        }
        if (slot == GPU_FABRIC_MAX_NODES) {
            gpu_fabric_end(fabric, daif);
            return false;
        }
        node = &fabric->nodes[slot];
        if (node->generation == 0U)
            node->generation = 1U;
        else if (!gpu_fabric_generation_bump(&node->generation)) {
            gpu_fabric_end(fabric, daif);
            return false;
        }
        generation = node->generation;
    }
    gpu_fabric_zero(node, sizeof(*node));
    node->epoch = facts->epoch;
    node->supported_kernel_mask = facts->supported_kernel_mask;
    node->verified_vram_bytes = facts->verified_vram_bytes;
    node->resident_capacity_bytes = facts->resident_capacity_bytes;
    node->resident_used_bytes = facts->resident_used_bytes;
    node->node_id = facts->node_id;
    node->generation = generation;
    node->health = (u8)facts->health;
    node->load_per_mille = (u16)facts->load_per_mille;
    node->gpu_verified = gpu_verified ? 1U : 0U;
    node->state = gpu_verified && facts->health == GPU_FABRIC_HEALTH_OK ?
        GPU_FABRIC_NODE_ELIGIBLE : GPU_FABRIC_NODE_UNVERIFIED;
    node_out->epoch = node->epoch;
    node_out->controller_id = fabric->owner.controller_id;
    node_out->node_id = node->node_id;
    node_out->generation = node->generation;
    node_out->_reserved = 0U;
    gpu_fabric_end(fabric, daif);
    return true;
}

bool gpu_fabric_node_health_report(struct gpu_fabric *fabric,
                                   const struct gpu_fabric_node_handle *handle,
                                   enum gpu_fabric_health health,
                                   u32 load_per_mille, u32 caller_core)
{
    u64 daif;
    u32 slot;
    struct gpu_fabric_node_record *node;

    if (!gpu_fabric_begin(fabric, caller_core, &daif))
        return false;
    if (!gpu_fabric_health_valid(health) || load_per_mille > 1000U ||
        !gpu_fabric_node_handle_valid(fabric, handle, &slot)) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    node = &fabric->nodes[slot];
    node->health = (u8)health;
    node->load_per_mille = (u16)load_per_mille;
    if (health == GPU_FABRIC_HEALTH_FAILED) {
        gpu_fabric_invalidate_node_placements(fabric, node->node_id);
        node->reserved_bytes = 0U;
        if (gpu_fabric_generation_bump(&node->generation))
            node->state = GPU_FABRIC_NODE_QUARANTINED;
        else
            node->state = GPU_FABRIC_NODE_REMOVED;
    } else {
        node->state = node->gpu_verified != 0U &&
                      health == GPU_FABRIC_HEALTH_OK ?
            GPU_FABRIC_NODE_ELIGIBLE : GPU_FABRIC_NODE_UNVERIFIED;
    }
    gpu_fabric_end(fabric, daif);
    return true;
}

bool gpu_fabric_node_remove(struct gpu_fabric *fabric,
                            const struct gpu_fabric_node_handle *handle,
                            u32 caller_core)
{
    u64 daif;
    u32 slot;
    struct gpu_fabric_node_record *node;

    if (!gpu_fabric_begin(fabric, caller_core, &daif))
        return false;
    if (!gpu_fabric_node_handle_valid(fabric, handle, &slot)) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    node = &fabric->nodes[slot];
    gpu_fabric_invalidate_node_placements(fabric, node->node_id);
    node->reserved_bytes = 0U;
    node->gpu_verified = 0U;
    node->health = GPU_FABRIC_HEALTH_FAILED;
    node->state = gpu_fabric_generation_bump(&node->generation) ?
        GPU_FABRIC_NODE_REMOVED : GPU_FABRIC_NODE_QUARANTINED;
    gpu_fabric_end(fabric, daif);
    return true;
}

bool gpu_fabric_node_get(const struct gpu_fabric *fabric, u32 node_id,
                         struct gpu_fabric_node_facts *facts_out,
                         u32 *generation_out)
{
    u32 slot;
    const struct gpu_fabric_node_record *node;

    if (!fabric || !facts_out || !generation_out || node_id == 0U)
        return false;
    slot = gpu_fabric_node_slot(fabric, node_id);
    if (slot == GPU_FABRIC_MAX_NODES)
        return false;
    node = &fabric->nodes[slot];
    facts_out->node_id = node->node_id;
    facts_out->_reserved = 0U;
    facts_out->epoch = node->epoch;
    facts_out->supported_kernel_mask = node->supported_kernel_mask;
    facts_out->verified_vram_bytes = node->verified_vram_bytes;
    facts_out->resident_capacity_bytes = node->resident_capacity_bytes;
    facts_out->resident_used_bytes = node->resident_used_bytes;
    facts_out->health = node->health;
    facts_out->load_per_mille = node->load_per_mille;
    *generation_out = node->generation;
    return true;
}

bool gpu_fabric_placement_create(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_request *request, u32 caller_core,
    struct gpu_fabric_placement_handle *placement_out)
{
    u64 daif;
    u32 chosen[GPU_FABRIC_MAX_SHARDS];
    u64 planned[GPU_FABRIC_MAX_NODES] = { 0U };
    u32 placement_slot;
    u32 i;
    u32 generation;

    gpu_fabric_clear_placement_handle(placement_out);
    if (!gpu_fabric_begin(fabric, caller_core, &daif))
        return false;
    if (!placement_out || !request || request->model_id == 0U ||
        request->model_generation == 0U || request->shard_count == 0U ||
        request->shard_count > GPU_FABRIC_MAX_SHARDS ||
        request->_reserved != 0U) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    for (placement_slot = 0U; placement_slot < GPU_FABRIC_MAX_PLACEMENTS;
         placement_slot++) {
        if (fabric->placements[placement_slot].state ==
            GPU_FABRIC_PLACEMENT_EMPTY ||
            fabric->placements[placement_slot].state ==
            GPU_FABRIC_PLACEMENT_RELEASED)
            break;
    }
    if (placement_slot == GPU_FABRIC_MAX_PLACEMENTS) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    for (i = 0U; i < request->shard_count; i++) {
        const struct gpu_fabric_shard_request *shard = &request->shards[i];
        u32 node_slot;
        u32 candidate = GPU_FABRIC_MAX_NODES;
        u32 other;

        if (shard->shard_id == 0U || shard->bytes == 0U ||
            shard->_reserved != 0U ||
            shard->required_kernel_mask == 0U) {
            gpu_fabric_end(fabric, daif);
            return false;
        }
        for (other = 0U; other < i; other++) {
            if (request->shards[other].shard_id == shard->shard_id) {
                gpu_fabric_end(fabric, daif);
                return false;
            }
        }
        for (node_slot = 0U; node_slot < GPU_FABRIC_MAX_NODES; node_slot++) {
            const struct gpu_fabric_node_record *node =
                &fabric->nodes[node_slot];
            u64 available;

            if (!gpu_fabric_node_eligible(node, shard->required_kernel_mask) ||
                (shard->affinity_node_id != 0U &&
                 shard->affinity_node_id != node->node_id))
                continue;
            available = gpu_fabric_node_available(node);
            if (planned[node_slot] > available ||
                shard->bytes > available - planned[node_slot])
                continue;
            if (candidate == GPU_FABRIC_MAX_NODES ||
                node->node_id < fabric->nodes[candidate].node_id)
                candidate = node_slot;
        }
        if (candidate == GPU_FABRIC_MAX_NODES ||
            planned[candidate] > ~0ULL - shard->bytes) {
            gpu_fabric_end(fabric, daif);
            return false;
        }
        chosen[i] = candidate;
        planned[candidate] += shard->bytes;
    }
    if (!gpu_fabric_generation_bump(
            &fabric->placements[placement_slot].generation)) {
        fabric->placements[placement_slot].state = GPU_FABRIC_PLACEMENT_RETIRED;
        gpu_fabric_end(fabric, daif);
        return false;
    }
    generation = fabric->placements[placement_slot].generation;
    gpu_fabric_zero(&fabric->placements[placement_slot],
                    sizeof(fabric->placements[placement_slot]));
    fabric->placements[placement_slot].generation = generation;
    fabric->placements[placement_slot].model_id = request->model_id;
    fabric->placements[placement_slot].model_generation = request->model_generation;
    fabric->placements[placement_slot].state = GPU_FABRIC_PLACEMENT_ACTIVE;
    fabric->placements[placement_slot].shard_count = request->shard_count;
    gpu_fabric_zero(fabric->flows[placement_slot],
                    sizeof(fabric->flows[placement_slot]));
    for (i = 0U; i < request->shard_count; i++) {
        const struct gpu_fabric_shard_request *source = &request->shards[i];
        struct gpu_fabric_shard_record *destination =
            &fabric->shards[placement_slot][i];
        struct gpu_fabric_node_record *node = &fabric->nodes[chosen[i]];

        gpu_fabric_zero(destination, sizeof(*destination));
        destination->shard_id = source->shard_id;
        destination->bytes = source->bytes;
        destination->required_kernel_mask = source->required_kernel_mask;
        destination->affinity_node_id = source->affinity_node_id;
        destination->node_id = node->node_id;
        destination->node_generation = node->generation;
        node->reserved_bytes += source->bytes;
    }
    placement_out->model_id = request->model_id;
    placement_out->model_generation = request->model_generation;
    placement_out->controller_id = fabric->owner.controller_id;
    placement_out->slot = placement_slot;
    placement_out->generation = fabric->placements[placement_slot].generation;
    placement_out->_reserved = 0U;
    gpu_fabric_end(fabric, daif);
    return true;
}

bool gpu_fabric_placement_release(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_handle *placement, u32 caller_core)
{
    u64 daif;
    u32 slot;

    if (!gpu_fabric_begin(fabric, caller_core, &daif))
        return false;
    if (!gpu_fabric_placement_handle_valid(fabric, placement, &slot) ||
        fabric->placements[slot].state == GPU_FABRIC_PLACEMENT_EMPTY ||
        fabric->placements[slot].state == GPU_FABRIC_PLACEMENT_RETIRED) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    if (fabric->placements[slot].state == GPU_FABRIC_PLACEMENT_ACTIVE)
        gpu_fabric_invalidate_placement(
            fabric, slot, GPU_FABRIC_PLACEMENT_RELEASED);
    else
        fabric->placements[slot].state = GPU_FABRIC_PLACEMENT_RELEASED;
    if (!gpu_fabric_generation_bump(&fabric->placements[slot].generation))
        fabric->placements[slot].state = GPU_FABRIC_PLACEMENT_RETIRED;
    gpu_fabric_end(fabric, daif);
    return true;
}

bool gpu_fabric_placement_state_get(
    const struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_handle *placement,
    enum gpu_fabric_placement_state *state_out)
{
    u32 slot;

    if (!state_out || !gpu_fabric_placement_handle_valid(
            fabric, placement, &slot))
        return false;
    *state_out = (enum gpu_fabric_placement_state)fabric->placements[slot].state;
    return true;
}

bool gpu_fabric_shard_assignment_get(
    const struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_handle *placement, u64 shard_id,
    u32 *node_id_out, u32 *node_generation_out)
{
    u32 placement_slot;
    u32 shard_slot;

    if (!node_id_out || !node_generation_out ||
        !gpu_fabric_placement_active(fabric, placement, &placement_slot) ||
        shard_id == 0U)
        return false;
    shard_slot = gpu_fabric_shard_slot(fabric, placement_slot, shard_id);
    if (shard_slot == GPU_FABRIC_MAX_SHARDS)
        return false;
    *node_id_out = fabric->shards[placement_slot][shard_slot].node_id;
    *node_generation_out =
        fabric->shards[placement_slot][shard_slot].node_generation;
    return true;
}

bool gpu_fabric_activation_publish(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_request *request, u32 caller_core,
    struct gpu_fabric_activation_handle *activation_out)
{
    u64 daif;
    u32 placement_slot;
    u32 source_slot;
    u32 target_slot;
    u32 source_node_slot;
    u32 target_node_slot;
    u32 flow_slot;
    u32 activation_slot;
    struct gpu_fabric_flow_record *flow;

    gpu_fabric_clear_activation_handle(activation_out);
    if (!gpu_fabric_begin(fabric, caller_core, &daif))
        return false;
    if (!activation_out || !request ||
        !gpu_fabric_placement_active(fabric, &request->placement,
                                     &placement_slot) ||
        request->source_shard_id == 0U || request->target_shard_id == 0U ||
        request->source_shard_id == request->target_shard_id ||
        request->sequence == 0U || request->span_bytes == 0U ||
        request->span_start > ~0ULL - request->span_bytes) {
        if (request && gpu_fabric_placement_handle_valid(
                fabric, &request->placement, &placement_slot))
            gpu_fabric_invalidate_placement(
                fabric, placement_slot, GPU_FABRIC_PLACEMENT_QUARANTINED);
        gpu_fabric_end(fabric, daif);
        return false;
    }
    source_slot = gpu_fabric_shard_slot(fabric, placement_slot,
                                        request->source_shard_id);
    target_slot = gpu_fabric_shard_slot(fabric, placement_slot,
                                        request->target_shard_id);
    if (source_slot == GPU_FABRIC_MAX_SHARDS ||
        target_slot == GPU_FABRIC_MAX_SHARDS ||
        fabric->shards[placement_slot][source_slot].node_id !=
            request->source_node_id ||
        fabric->shards[placement_slot][source_slot].node_generation !=
            request->source_node_generation ||
        fabric->shards[placement_slot][target_slot].node_id !=
            request->target_node_id ||
        fabric->shards[placement_slot][target_slot].node_generation !=
            request->target_node_generation) {
        gpu_fabric_invalidate_placement(
            fabric, placement_slot, GPU_FABRIC_PLACEMENT_QUARANTINED);
        gpu_fabric_end(fabric, daif);
        return false;
    }
    source_node_slot = gpu_fabric_node_slot_generation(
        fabric, request->source_node_id, request->source_node_generation);
    target_node_slot = gpu_fabric_node_slot_generation(
        fabric, request->target_node_id, request->target_node_generation);
    if (source_node_slot == GPU_FABRIC_MAX_NODES ||
        target_node_slot == GPU_FABRIC_MAX_NODES ||
        !gpu_fabric_node_eligible(&fabric->nodes[source_node_slot], 0U) ||
        !gpu_fabric_node_eligible(&fabric->nodes[target_node_slot], 0U)) {
        gpu_fabric_invalidate_placement(
            fabric, placement_slot, GPU_FABRIC_PLACEMENT_QUARANTINED);
        gpu_fabric_end(fabric, daif);
        return false;
    }
    flow_slot = gpu_fabric_flow_slot(fabric, placement_slot,
                                     request->source_shard_id,
                                     request->target_shard_id);
    if (flow_slot != GPU_FABRIC_MAX_FLOWS) {
        flow = &fabric->flows[placement_slot][flow_slot];
        if (flow->last_sequence == ~0ULL ||
            request->sequence != flow->last_sequence + 1U) {
            gpu_fabric_invalidate_placement(
                fabric, placement_slot, GPU_FABRIC_PLACEMENT_QUARANTINED);
            gpu_fabric_end(fabric, daif);
            return false;
        }
    } else {
        if (request->sequence != 1U) {
            gpu_fabric_invalidate_placement(
                fabric, placement_slot, GPU_FABRIC_PLACEMENT_QUARANTINED);
            gpu_fabric_end(fabric, daif);
            return false;
        }
        for (flow_slot = 0U; flow_slot < GPU_FABRIC_MAX_FLOWS; flow_slot++) {
            if (fabric->flows[placement_slot][flow_slot].in_use == 0U)
                break;
        }
        if (flow_slot == GPU_FABRIC_MAX_FLOWS) {
            gpu_fabric_end(fabric, daif);
            return false;
        }
        flow = &fabric->flows[placement_slot][flow_slot];
    }
    for (activation_slot = 0U; activation_slot < GPU_FABRIC_MAX_ACTIVATIONS;
         activation_slot++) {
        if (fabric->activations[activation_slot].state ==
                GPU_FABRIC_ACTIVATION_EMPTY ||
            fabric->activations[activation_slot].state ==
                GPU_FABRIC_ACTIVATION_RELEASED ||
            fabric->activations[activation_slot].state ==
                GPU_FABRIC_ACTIVATION_INVALID ||
            fabric->activations[activation_slot].state ==
                GPU_FABRIC_ACTIVATION_QUARANTINED)
            break;
    }
    if (activation_slot == GPU_FABRIC_MAX_ACTIVATIONS) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    if (!gpu_fabric_generation_bump(
            &fabric->activations[activation_slot].generation)) {
        fabric->activations[activation_slot].state =
            GPU_FABRIC_ACTIVATION_RETIRED;
        gpu_fabric_end(fabric, daif);
        return false;
    }
    flow->source_shard_id = request->source_shard_id;
    flow->target_shard_id = request->target_shard_id;
    flow->last_sequence = request->sequence;
    flow->generation++;
    flow->in_use = 1U;
    fabric->activations[activation_slot].placement_slot = placement_slot;
    fabric->activations[activation_slot].placement_generation =
        request->placement.generation;
    fabric->activations[activation_slot].state =
        request->span_bytes <= request->credit_capacity_bytes ?
            GPU_FABRIC_ACTIVATION_READY :
            GPU_FABRIC_ACTIVATION_BACKPRESSURED;
    fabric->routes[activation_slot].source_shard_id = request->source_shard_id;
    fabric->routes[activation_slot].target_shard_id = request->target_shard_id;
    fabric->routes[activation_slot].sequence = request->sequence;
    fabric->routes[activation_slot].source_node_id = request->source_node_id;
    fabric->routes[activation_slot].source_node_generation =
        request->source_node_generation;
    fabric->routes[activation_slot].target_node_id = request->target_node_id;
    fabric->routes[activation_slot].target_node_generation =
        request->target_node_generation;
    fabric->routes[activation_slot].placement_slot = placement_slot;
    fabric->routes[activation_slot].placement_generation =
        request->placement.generation;
    fabric->spans[activation_slot].span_start = request->span_start;
    fabric->spans[activation_slot].span_bytes = request->span_bytes;
    fabric->spans[activation_slot].credit_capacity_bytes =
        request->credit_capacity_bytes;
    activation_out->sequence = request->sequence;
    activation_out->controller_id = fabric->owner.controller_id;
    activation_out->slot = activation_slot;
    activation_out->generation = fabric->activations[activation_slot].generation;
    activation_out->_reserved = 0U;
    gpu_fabric_end(fabric, daif);
    return true;
}

bool gpu_fabric_activation_retry(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_handle *activation,
    u64 credit_capacity_bytes, u32 caller_core)
{
    u64 daif;
    u32 slot;

    if (!gpu_fabric_begin(fabric, caller_core, &daif))
        return false;
    if (!gpu_fabric_activation_handle_valid(fabric, activation, &slot) ||
        fabric->activations[slot].state !=
            GPU_FABRIC_ACTIVATION_BACKPRESSURED) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    if (fabric->spans[slot].span_bytes <= credit_capacity_bytes)
        fabric->activations[slot].state = GPU_FABRIC_ACTIVATION_READY;
    gpu_fabric_end(fabric, daif);
    return true;
}

bool gpu_fabric_activation_get(
    const struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_handle *activation,
    struct gpu_fabric_activation_descriptor *descriptor_out)
{
    u32 slot;

    if (!descriptor_out || !gpu_fabric_activation_handle_valid(
            fabric, activation, &slot))
        return false;
    descriptor_out->source_shard_id = fabric->routes[slot].source_shard_id;
    descriptor_out->target_shard_id = fabric->routes[slot].target_shard_id;
    descriptor_out->sequence = fabric->routes[slot].sequence;
    descriptor_out->span_start = fabric->spans[slot].span_start;
    descriptor_out->span_bytes = fabric->spans[slot].span_bytes;
    descriptor_out->credit_capacity_bytes =
        fabric->spans[slot].credit_capacity_bytes;
    descriptor_out->source_node_id = fabric->routes[slot].source_node_id;
    descriptor_out->source_node_generation =
        fabric->routes[slot].source_node_generation;
    descriptor_out->target_node_id = fabric->routes[slot].target_node_id;
    descriptor_out->target_node_generation =
        fabric->routes[slot].target_node_generation;
    descriptor_out->state = fabric->activations[slot].state;
    descriptor_out->_reserved = 0U;
    return true;
}

bool gpu_fabric_activation_dispatch(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_handle *activation, u32 caller_core,
    struct gpu_fabric_activation_descriptor *descriptor_out)
{
    u64 daif;
    u32 slot;
    bool ok;

    if (!gpu_fabric_begin(fabric, caller_core, &daif))
        return false;
    if (!gpu_fabric_activation_handle_valid(fabric, activation, &slot) ||
        fabric->activations[slot].state != GPU_FABRIC_ACTIVATION_READY) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    ok = gpu_fabric_activation_get(fabric, activation, descriptor_out);
    if (ok)
        fabric->activations[slot].state = GPU_FABRIC_ACTIVATION_DISPATCHED;
    gpu_fabric_end(fabric, daif);
    return ok;
}

bool gpu_fabric_activation_release(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_handle *activation, u32 caller_core)
{
    u64 daif;
    u32 slot;

    if (!gpu_fabric_begin(fabric, caller_core, &daif))
        return false;
    if (!gpu_fabric_activation_handle_valid(fabric, activation, &slot) ||
        fabric->activations[slot].state != GPU_FABRIC_ACTIVATION_DISPATCHED) {
        gpu_fabric_end(fabric, daif);
        return false;
    }
    gpu_fabric_activation_invalidate_slot(
        fabric, slot, GPU_FABRIC_ACTIVATION_RELEASED);
    gpu_fabric_end(fabric, daif);
    return true;
}

bool gpu_fabric_hardware_enable_allowed(const struct gpu_fabric *fabric,
                                        u32 caller_core)
{
    (void)fabric;
    (void)caller_core;
    return false;
}
