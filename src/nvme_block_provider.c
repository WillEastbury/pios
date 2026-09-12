/*
 * nvme_block_provider.c - offline synchronous namespace block provider.
 *
 * This module consumes only callback spans. It contains no controller,
 * transport, allocation, or storage-consumer integration.
 */
#include "types.h"
#include "nvme_block_provider.h"

static void zero_bytes(void *ptr, usize bytes)
{
    u8 *out = (u8 *)ptr;
    usize i;

    for (i = 0U; i < bytes; i++)
        out[i] = 0U;
}

static bool bytes_zero(const void *ptr, usize bytes)
{
    const u8 *in = (const u8 *)ptr;
    usize i;

    for (i = 0U; i < bytes; i++) {
        if (in[i] != 0U)
            return false;
    }
    return true;
}

static bool geometry_valid(const struct nvme_block_provider_geometry *geometry)
{
    u32 i;

    if (!geometry || geometry->logical_blocks == 0U ||
        geometry->namespace_id == 0U || geometry->namespace_generation == 0U ||
        geometry->instance_epoch == 0U || geometry->read_only > 1U ||
        geometry->max_transfer_blocks == 0U ||
        (u64)geometry->max_transfer_blocks > geometry->logical_blocks ||
        geometry->block_bytes < NVME_BLOCK_PROVIDER_BLOCK_BYTES_MIN ||
        geometry->block_bytes > NVME_BLOCK_PROVIDER_BLOCK_BYTES_MAX ||
        (geometry->block_bytes & (geometry->block_bytes - 1U)) != 0U ||
        geometry->logical_blocks > ~0ULL / geometry->block_bytes ||
        geometry->capacity_bytes !=
            geometry->logical_blocks * (u64)geometry->block_bytes)
        return false;
    for (i = 0U; i < sizeof(geometry->_reserved); i++) {
        if (geometry->_reserved[i] != 0U)
            return false;
    }
    return true;
}

static u64 geometry_fingerprint(const struct nvme_block_provider_geometry *geometry)
{
    u64 value = geometry->logical_blocks ^ geometry->capacity_bytes;

    value ^= geometry->namespace_generation;
    value ^= geometry->instance_epoch;
    value ^= ((u64)geometry->namespace_id << 32) | geometry->block_bytes;
    value ^= (u64)geometry->max_transfer_blocks << 17;
    value ^= (u64)geometry->read_only << 63;
    return value ^ NVME_BLOCK_PROVIDER_MAGIC;
}

static void handle_clear(struct nvme_block_provider_handle *handle)
{
    if (handle)
        zero_bytes(handle, sizeof(*handle));
}

static void span_clear(struct nvme_block_provider_span *span)
{
    if (span)
        span->used = 0U;
}

static void handle_set(const struct nvme_block_provider *provider,
                       struct nvme_block_provider_handle *handle)
{
    *handle = (struct nvme_block_provider_handle) {
        .magic = NVME_BLOCK_PROVIDER_HANDLE_MAGIC,
        .instance_epoch = provider->geometry.instance_epoch,
        .namespace_generation = provider->geometry.namespace_generation,
        .provider_generation = provider->control.provider_generation,
        .namespace_id = provider->geometry.namespace_id,
    };
}

static bool provider_identity_valid(const struct nvme_block_provider *provider)
{
    return provider && provider->control.magic == NVME_BLOCK_PROVIDER_MAGIC &&
           geometry_valid(&provider->geometry) &&
           provider->control.geometry_fingerprint ==
               geometry_fingerprint(&provider->geometry);
}

static enum nvme_block_provider_result provider_base_check(
    const struct nvme_block_provider *provider, u32 caller_core)
{
    if (!provider)
        return NVME_BLOCK_PROVIDER_INVALID;
    if (provider->control.magic != NVME_BLOCK_PROVIDER_MAGIC)
        return NVME_BLOCK_PROVIDER_INVALID;
    if (caller_core != NVME_BLOCK_PROVIDER_OWNER_CORE ||
        core_id() != NVME_BLOCK_PROVIDER_OWNER_CORE ||
        provider->control.owner_core != caller_core)
        return NVME_BLOCK_PROVIDER_OWNER;
    if (!provider_identity_valid(provider))
        return NVME_BLOCK_PROVIDER_CORRUPT;
    if (provider->control.state == NVME_BLOCK_PROVIDER_RETIRED)
        return NVME_BLOCK_PROVIDER_RETIRED_RESULT;
    return NVME_BLOCK_PROVIDER_OK;
}

static bool handle_matches(const struct nvme_block_provider *provider,
                           const struct nvme_block_provider_handle *handle)
{
    return handle &&
           handle->magic == NVME_BLOCK_PROVIDER_HANDLE_MAGIC &&
           handle->instance_epoch == provider->geometry.instance_epoch &&
           handle->namespace_generation ==
               provider->geometry.namespace_generation &&
           handle->provider_generation ==
               provider->control.provider_generation &&
           handle->namespace_id == provider->geometry.namespace_id;
}

static void provider_quarantine(struct nvme_block_provider *provider,
                                enum nvme_block_provider_fault fault,
                                bool removed)
{
    provider->control.fault = fault;
    if (provider->control.provider_generation == ~0ULL) {
        provider->control.state = NVME_BLOCK_PROVIDER_RETIRED;
        return;
    }
    provider->control.provider_generation++;
    provider->control.state = removed ? NVME_BLOCK_PROVIDER_REMOVED :
                                        NVME_BLOCK_PROVIDER_QUARANTINED;
}

static enum nvme_block_provider_result provider_operation_check(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle, u32 caller_core)
{
    enum nvme_block_provider_result result =
        provider_base_check(provider, caller_core);

    if (result != NVME_BLOCK_PROVIDER_OK)
        return result;
    if (!handle_matches(provider, handle))
        return NVME_BLOCK_PROVIDER_STALE;
    if (provider->control.state != NVME_BLOCK_PROVIDER_ACTIVE)
        return NVME_BLOCK_PROVIDER_INACTIVE;
    if (provider->control.callback_active != 0U)
        return NVME_BLOCK_PROVIDER_BUSY;
    return NVME_BLOCK_PROVIDER_OK;
}

static enum nvme_block_provider_result transfer(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle, u32 caller_core,
    u64 lba, u32 block_count, struct nvme_block_provider_span *span,
    bool is_write)
{
    enum nvme_block_provider_result result;
    nvme_block_provider_transfer_fn callback;
    struct nvme_block_provider_span callback_span;
    u64 requested;
    bool callback_ok;

    span_clear(span);
    result = provider_operation_check(provider, handle, caller_core);
    if (result != NVME_BLOCK_PROVIDER_OK)
        return result;
    if (!span || !span->bytes || block_count == 0U ||
        block_count > provider->geometry.max_transfer_blocks ||
        lba >= provider->geometry.logical_blocks ||
        (u64)block_count > provider->geometry.logical_blocks - lba ||
        (u64)block_count > ~0ULL / provider->geometry.block_bytes)
        return NVME_BLOCK_PROVIDER_RANGE;
    requested = (u64)block_count * provider->geometry.block_bytes;
    if (span->capacity < requested)
        return NVME_BLOCK_PROVIDER_RANGE;
    if (is_write && provider->geometry.read_only != 0U)
        return NVME_BLOCK_PROVIDER_READ_ONLY;
    callback = is_write ? provider->backend.write : provider->backend.read;
    if (!callback)
        return NVME_BLOCK_PROVIDER_READ_ONLY;

    callback_span = (struct nvme_block_provider_span) {
        .bytes = span->bytes,
        .capacity = requested,
        .used = 0U,
    };
    provider->control.callback_active = 1U;
    callback_ok = callback(provider->backend.context, lba, block_count,
                           &callback_span);
    provider->control.callback_active = 0U;
    if (!callback_ok) {
        provider_quarantine(provider, NVME_BLOCK_PROVIDER_FAULT_CALLBACK,
                            false);
        return NVME_BLOCK_PROVIDER_IO;
    }
    if (callback_span.bytes != span->bytes ||
        callback_span.capacity != requested ||
        callback_span.used != requested) {
        provider_quarantine(provider, NVME_BLOCK_PROVIDER_FAULT_PARTIAL,
                            false);
        return NVME_BLOCK_PROVIDER_PARTIAL;
    }
    span->used = requested;
    if (provider->control.completed_operations == ~0ULL) {
        provider_quarantine(provider, NVME_BLOCK_PROVIDER_FAULT_CORRUPT,
                            false);
        span->used = 0U;
        return NVME_BLOCK_PROVIDER_CORRUPT;
    }
    provider->control.completed_operations++;
    return NVME_BLOCK_PROVIDER_OK;
}

enum nvme_block_provider_result nvme_block_provider_init(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_geometry *geometry,
    const struct nvme_block_provider_callbacks *callbacks, u32 caller_core,
    struct nvme_block_provider_handle *handle_out)
{
    handle_clear(handle_out);
    if (!provider || !geometry || !callbacks ||
        caller_core != NVME_BLOCK_PROVIDER_OWNER_CORE ||
        core_id() != NVME_BLOCK_PROVIDER_OWNER_CORE ||
        !geometry_valid(geometry) || !callbacks->read ||
        (!geometry->read_only && !callbacks->write) ||
        !bytes_zero(provider, sizeof(*provider)))
        return NVME_BLOCK_PROVIDER_INVALID;

    provider->control.magic = NVME_BLOCK_PROVIDER_MAGIC;
    provider->control.provider_generation = geometry->namespace_generation;
    provider->control.geometry_fingerprint = geometry_fingerprint(geometry);
    provider->control.owner_core = NVME_BLOCK_PROVIDER_OWNER_CORE;
    provider->control.state = NVME_BLOCK_PROVIDER_ACTIVE;
    provider->geometry = *geometry;
    provider->backend.read = callbacks->read;
    provider->backend.write = callbacks->write;
    provider->backend.context = callbacks->context;
    handle_set(provider, handle_out);
    return NVME_BLOCK_PROVIDER_OK;
}

enum nvme_block_provider_result nvme_block_provider_open(
    const struct nvme_block_provider *provider, u32 caller_core,
    struct nvme_block_provider_handle *handle_out)
{
    enum nvme_block_provider_result result;

    handle_clear(handle_out);
    if (!handle_out)
        return NVME_BLOCK_PROVIDER_INVALID;
    result = provider_base_check(provider, caller_core);
    if (result != NVME_BLOCK_PROVIDER_OK)
        return result;
    if (provider->control.state != NVME_BLOCK_PROVIDER_ACTIVE)
        return NVME_BLOCK_PROVIDER_INACTIVE;
    if (provider->control.callback_active != 0U)
        return NVME_BLOCK_PROVIDER_BUSY;
    handle_set(provider, handle_out);
    return NVME_BLOCK_PROVIDER_OK;
}

enum nvme_block_provider_result nvme_block_provider_geometry_get(
    const struct nvme_block_provider *provider, u32 caller_core,
    struct nvme_block_provider_geometry *geometry_out)
{
    enum nvme_block_provider_result result;

    if (geometry_out)
        zero_bytes(geometry_out, sizeof(*geometry_out));
    if (!geometry_out)
        return NVME_BLOCK_PROVIDER_INVALID;
    result = provider_base_check(provider, caller_core);
    if (result != NVME_BLOCK_PROVIDER_OK)
        return result;
    *geometry_out = provider->geometry;
    return NVME_BLOCK_PROVIDER_OK;
}

enum nvme_block_provider_result nvme_block_provider_status_get(
    const struct nvme_block_provider *provider, u32 caller_core,
    struct nvme_block_provider_status *status_out)
{
    if (status_out)
        zero_bytes(status_out, sizeof(*status_out));
    if (!status_out || !provider ||
        provider->control.magic != NVME_BLOCK_PROVIDER_MAGIC)
        return NVME_BLOCK_PROVIDER_INVALID;
    if (caller_core != NVME_BLOCK_PROVIDER_OWNER_CORE ||
        core_id() != NVME_BLOCK_PROVIDER_OWNER_CORE ||
        provider->control.owner_core != caller_core)
        return NVME_BLOCK_PROVIDER_OWNER;
    if (!provider_identity_valid(provider))
        return NVME_BLOCK_PROVIDER_CORRUPT;
    if (provider->control.callback_active != 0U)
        return NVME_BLOCK_PROVIDER_BUSY;
    *status_out = (struct nvme_block_provider_status) {
        .instance_epoch = provider->geometry.instance_epoch,
        .namespace_generation = provider->geometry.namespace_generation,
        .provider_generation = provider->control.provider_generation,
        .capacity_bytes = provider->geometry.capacity_bytes,
        .namespace_id = provider->geometry.namespace_id,
        .block_bytes = provider->geometry.block_bytes,
        .max_transfer_blocks = provider->geometry.max_transfer_blocks,
        .state = provider->control.state,
        .fault = provider->control.fault,
        .owner_core = provider->control.owner_core,
        .read_only = provider->geometry.read_only,
    };
    return NVME_BLOCK_PROVIDER_OK;
}

enum nvme_block_provider_result nvme_block_provider_read(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle, u32 caller_core,
    u64 lba, u32 block_count, struct nvme_block_provider_span *span)
{
    return transfer(provider, handle, caller_core, lba, block_count, span,
                    false);
}

enum nvme_block_provider_result nvme_block_provider_write(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle, u32 caller_core,
    u64 lba, u32 block_count, struct nvme_block_provider_span *span)
{
    return transfer(provider, handle, caller_core, lba, block_count, span,
                    true);
}

enum nvme_block_provider_result nvme_block_provider_revoke(
    struct nvme_block_provider *provider, u32 caller_core,
    enum nvme_block_provider_revoke_reason reason)
{
    enum nvme_block_provider_result result =
        provider_base_check(provider, caller_core);
    enum nvme_block_provider_fault fault;
    bool removed;

    if (result != NVME_BLOCK_PROVIDER_OK)
        return result;
    if (provider->control.state != NVME_BLOCK_PROVIDER_ACTIVE)
        return NVME_BLOCK_PROVIDER_INACTIVE;
    if (provider->control.callback_active != 0U)
        return NVME_BLOCK_PROVIDER_BUSY;
    if (reason == NVME_BLOCK_PROVIDER_REVOKE_TIMEOUT) {
        fault = NVME_BLOCK_PROVIDER_FAULT_TIMEOUT;
        removed = false;
    } else if (reason == NVME_BLOCK_PROVIDER_REVOKE_AER) {
        fault = NVME_BLOCK_PROVIDER_FAULT_AER;
        removed = false;
    } else if (reason == NVME_BLOCK_PROVIDER_REVOKE_REMOVAL) {
        fault = NVME_BLOCK_PROVIDER_FAULT_REMOVED;
        removed = true;
    } else {
        return NVME_BLOCK_PROVIDER_INVALID;
    }
    provider_quarantine(provider, fault, removed);
    return NVME_BLOCK_PROVIDER_OK;
}
