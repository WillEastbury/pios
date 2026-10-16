#include "tensor_broker.h"

_Static_assert(sizeof(struct tensor_descriptor) == 128,
               "tensor descriptor ABI stride");
_Static_assert(sizeof(struct tb_capability) == 64,
               "backend capability isolation");
_Static_assert(sizeof(struct tb_backend) == 128,
               "backend record isolation");
_Static_assert(sizeof(struct tb_job) == 192,
               "job record isolation");

static struct tb_job *job_lookup(struct tensor_broker *b, u32 handle)
{
    u32 slot = handle & 7U;
    if (!b || !slot || slot > TB_DEPTH) return NULL;
    struct tb_job *j = &b->jobs[slot - 1U];
    return j->state && j->generation == (handle >> 3) ? j : NULL;
}

static bool capability_valid(const struct tb_capability *c)
{
    return c && c->backend_id && c->operation == TB_OP_MATVEC &&
        c->input_dtype == TB_DTYPE_I8 && c->output_dtype == TB_DTYPE_I32 &&
        c->layout == TB_LAYOUT_ROW_MAJOR && c->max_rows && c->max_cols &&
        c->col_multiple && c->capability && c->device_generation;
}

i32 tb_register(struct tensor_broker *b, const struct tb_capability *cap,
                tb_execute execute)
{
    if (!b || !capability_valid(cap) || !execute) return TB_INVALID;
    for (u32 i = 0; i < TB_BACKENDS; i++) {
        struct tb_backend *p = &b->backends[i];
        if (p->capability.live && p->capability.backend_id == cap->backend_id &&
            p->capability.device_generation == cap->device_generation)
            return TB_INVALID;
    }
    for (u32 i = 0; i < TB_BACKENDS; i++) {
        struct tb_backend *p = &b->backends[i];
        if (p->capability.live) continue;
        u32 generation = p->capability.backend_generation + 1U;
        if (!generation) return TB_INVALID;
        p->capability = *cap;
        p->capability.backend_generation = generation;
        p->capability.live = 1U;
        p->execute = execute;
        return TB_OK;
    }
    return TB_FULL;
}

i32 tb_retire(struct tensor_broker *b, u32 backend_id, u32 device_generation)
{
    if (!b || !backend_id || !device_generation) return TB_INVALID;
    for (u32 i = 0; i < TB_BACKENDS; i++) {
        struct tb_backend *p = &b->backends[i];
        if (p->capability.live && p->capability.backend_id == backend_id &&
            p->capability.device_generation == device_generation) {
            p->capability.live = 0;
            p->execute = NULL;
            return TB_OK;
        }
    }
    return TB_STALE;
}

static bool compatible(const struct tb_capability *c,
                       const struct tensor_descriptor *d)
{
    return c->live && c->operation == d->operation &&
        c->input_dtype == d->input_dtype &&
        c->output_dtype == d->output_dtype && c->layout == d->layout &&
        d->rows <= c->max_rows && d->cols <= c->max_cols &&
        !(d->cols % c->col_multiple);
}

static i32 resolve_descriptor(const struct tensor_descriptor *d,
                              struct lease_descriptor *matrix,
                              struct lease_descriptor *vector,
                              struct lease_descriptor *output)
{
    if (lease_resolve(&d->matrix, matrix) || lease_resolve(&d->vector, vector) ||
        lease_resolve(&d->output, output))
        return TB_STALE;
    if (lease_validate(matrix, d->owner_capsule) ||
        lease_validate(vector, d->owner_capsule) ||
        lease_validate(output, d->owner_capsule))
        return TB_AUTHORITY;
    u64 matrix_bytes = (u64)d->rows * d->cols;
    u64 output_bytes = (u64)d->rows * sizeof(i32);
    if (!matrix_bytes || matrix_bytes > matrix->length ||
        d->cols > vector->length || output_bytes > output->length ||
        matrix->state != LEASE_STATE_READY ||
        vector->state != LEASE_STATE_READY ||
        output->state != LEASE_STATE_WRITING ||
        !(matrix->flags & LEASE_F_READONLY) ||
        !(vector->flags & LEASE_F_READONLY) ||
        !(output->flags & LEASE_F_READWRITE))
        return TB_STATE;
    return TB_OK;
}

i32 tb_submit(struct tensor_broker *b, const struct tensor_descriptor *d,
              u32 *handle)
{
    struct lease_descriptor matrix, vector, output;
    if (!b || !d || !handle || d->abi_version != TB_ABI_VERSION ||
        d->operation != TB_OP_MATVEC || d->input_dtype != TB_DTYPE_I8 ||
        d->output_dtype != TB_DTYPE_I32 ||
        d->layout != TB_LAYOUT_ROW_MAJOR || !d->rows || !d->cols ||
        d->reserved || d->policy > TB_POLICY_EXACT ||
        (d->policy == TB_POLICY_EXACT &&
         (!d->target_backend || !d->target_device_generation)) ||
        (d->policy == TB_POLICY_AUTO &&
         (d->target_backend || d->target_device_generation)))
        return TB_INVALID;
    if (b->count == TB_DEPTH) return TB_FULL;
    i32 rc = resolve_descriptor(d, &matrix, &vector, &output);
    if (rc) return rc;

    u32 selected = TB_BACKENDS;
    for (u32 i = 0; i < TB_BACKENDS; i++) {
        const struct tb_capability *c = &b->backends[i].capability;
        if (!compatible(c, d) ||
            (d->policy == TB_POLICY_EXACT &&
             (c->backend_id != d->target_backend ||
              c->device_generation != d->target_device_generation)))
            continue;
        if (selected == TB_BACKENDS ||
            c->priority > b->backends[selected].capability.priority)
            selected = i;
    }
    if (selected == TB_BACKENDS) return TB_UNAVAILABLE;

    u32 slot;
    for (slot = 0; slot < TB_DEPTH; slot++)
        if (!b->jobs[slot].state &&
            b->jobs[slot].generation < 0x1fffffffU) break;
    if (slot == TB_DEPTH) return TB_FULL;
    struct tb_job *j = &b->jobs[slot];
    j->generation++;
    j->state = 1U;
    j->backend_slot = selected;
    j->backend_generation =
        b->backends[selected].capability.backend_generation;
    j->status = TB_OK;
    j->selected_backend =
        b->backends[selected].capability.backend_id;
    j->descriptor = *d;
    b->queue[(b->head + b->count) % TB_DEPTH] = slot;
    b->count++;
    *handle = (j->generation << 3) | (slot + 1U);
    return TB_OK;
}

bool tb_step(struct tensor_broker *b)
{
    if (!b || !b->count) return false;
    u32 slot = b->queue[b->head];
    struct tb_job *j = &b->jobs[slot];
    struct tb_backend *backend = &b->backends[j->backend_slot];
    struct lease_descriptor matrix, vector, output;

    if (!backend->capability.live ||
        backend->capability.backend_generation != j->backend_generation) {
        j->status = TB_STALE;
    } else {
        i32 rc = resolve_descriptor(&j->descriptor, &matrix, &vector, &output);
        if (rc) {
            j->status = rc;
        } else if (lease_begin_read(&matrix)) {
            j->status = TB_STATE;
        } else if (lease_begin_read(&vector)) {
            lease_end_read(&matrix);
            j->status = TB_STATE;
        } else {
            const i8 *a = (const i8 *)lease_ptr(&matrix);
            const i8 *x = (const i8 *)lease_ptr(&vector);
            i32 *out = (i32 *)lease_ptr(&output);
            if (!a || !x || !out ||
                !backend->execute(&j->descriptor, a, x, out))
                j->status = TB_EXECUTION;
            else if (lease_publish(&output))
                j->status = TB_STATE;
            lease_end_read(&matrix);
            lease_end_read(&vector);
        }
    }
    j->state = 2U;
    b->head = (b->head + 1U) % TB_DEPTH;
    b->count--;
    return true;
}

i32 tb_result(struct tensor_broker *b, u32 handle, u32 *selected_backend)
{
    struct tb_job *j = job_lookup(b, handle);
    if (!j) return TB_STALE;
    if (j->state != 2U || !selected_backend) return TB_INVALID;
    if (j->status) return j->status;
    *selected_backend = j->selected_backend;
    return TB_OK;
}

i32 tb_release(struct tensor_broker *b, u32 handle)
{
    struct tb_job *j = job_lookup(b, handle);
    if (!j) return TB_STALE;
    if (j->state != 2U) return TB_STATE;
    j->state = 0;
    j->backend_slot = 0;
    j->backend_generation = 0;
    j->status = TB_STALE;
    j->selected_backend = 0;
    for (u32 i = 0; i < sizeof(j->descriptor) / sizeof(u32); i++)
        ((u32 *)&j->descriptor)[i] = 0;
    return TB_OK;
}
