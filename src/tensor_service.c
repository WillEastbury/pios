#include "tensor_service.h"

_Static_assert(sizeof(struct ts_job) == 64, "job control isolation");
_Static_assert(sizeof(struct ts_backend) == 64, "backend isolation");
_Static_assert(sizeof(struct ts_payload) % 64 == 0, "payload stride");

static struct ts_job *lookup(struct tensor_service *s, u32 handle)
{
    u32 slot = handle & 7U;
    if (!s || !slot || slot > TS_DEPTH) return NULL;
    struct ts_job *j = &s->jobs[slot - 1U];
    return j->state && j->generation == (handle >> 3) ? j : NULL;
}

i32 ts_register(struct tensor_service *s, u32 id, ts_execute fn,
                u32 precision, u32 rows, u32 cols, u32 multiple)
{
    if (!s || id < TS_CPU || id > TS_QPU || !fn ||
        precision != TS_I8_I32 || !rows || rows > TS_ROWS ||
        !cols || cols > TS_COLS || !multiple) return TS_INVALID;
    struct ts_backend *b = &s->backends[id - 1U];
    if (b->live || b->generation == 0xffffffffU) return TS_INVALID;
    b->generation++;
    b->execute = fn;
    b->precision = precision;
    b->max_rows = rows;
    b->max_cols = cols;
    b->col_multiple = multiple;
    b->live = true;
    return TS_OK;
}

i32 ts_retire(struct tensor_service *s, u32 id)
{
    if (!s || id < TS_CPU || id > TS_QPU) return TS_INVALID;
    s->backends[id - 1U].live = false;
    s->backends[id - 1U].execute = NULL;
    return TS_OK;
}

static bool compatible(const struct ts_backend *b, u32 precision,
                       u32 rows, u32 cols)
{
    return b->live && b->precision == precision &&
        rows <= b->max_rows && cols <= b->max_cols &&
        !(cols % b->col_multiple);
}

i32 ts_submit(struct tensor_service *s, u32 pin, u32 precision,
              const i8 *a, u32 an, const i8 *x, u32 xn,
              u32 rows, u32 cols, u32 *handle)
{
    if (!s || !handle || !a || !x || pin > TS_QPU || !rows ||
        rows > TS_ROWS || !cols || cols > TS_COLS ||
        an < rows * cols || xn < cols) return TS_INVALID;
    if (s->count == TS_DEPTH) return TS_FULL;
    u32 id = pin;
    if (!id) {
        id = compatible(&s->backends[TS_QPU - 1U], precision, rows, cols)
            ? TS_QPU : TS_CPU;
    }
    if (!compatible(&s->backends[id - 1U], precision, rows, cols))
        return TS_UNAVAILABLE;
    u32 slot;
    for (slot = 0; slot < TS_DEPTH; slot++)
        if (!s->jobs[slot].state &&
            s->jobs[slot].generation < 0x1fffffffU) break;
    if (slot == TS_DEPTH) return TS_FULL;
    struct ts_job *j = &s->jobs[slot];
    struct ts_payload *p = &s->payload[slot];
    for (u32 i = 0; i < rows * cols; i++) p->matrix[i] = a[i];
    for (u32 i = 0; i < cols; i++) p->vector[i] = x[i];
    j->generation++;
    j->backend = id;
    j->backend_generation = s->backends[id - 1U].generation;
    j->rows = rows;
    j->cols = cols;
    j->status = TS_OK;
    j->state = 1;
    s->queue[(s->head + s->count) % TS_DEPTH] = slot;
    s->count++;
    *handle = (j->generation << 3) | (slot + 1U);
    return TS_OK;
}

bool ts_step(struct tensor_service *s)
{
    if (!s || !s->count) return false;
    u32 slot = s->queue[s->head];
    struct ts_job *j = &s->jobs[slot];
    struct ts_backend *b = &s->backends[j->backend - 1U];
    struct ts_payload *p = &s->payload[slot];
    if (!b->live || b->generation != j->backend_generation)
        j->status = TS_STALE;
    else if (!b->execute(p->matrix, p->vector, j->rows, j->cols, p->output))
        j->status = TS_EXECUTION;
    /* Never retry on another backend after execution may have had effects. */
    j->state = 2;
    s->head = (s->head + 1U) % TS_DEPTH;
    s->count--;
    return true;
}

i32 ts_result(struct tensor_service *s, u32 handle, i32 *out, u32 n)
{
    struct ts_job *j = lookup(s, handle);
    if (!j) return TS_STALE;
    if (j->state != 2 || !out || n < j->rows) return TS_INVALID;
    if (j->status) return j->status;
    struct ts_payload *p = &s->payload[(handle & 7U) - 1U];
    for (u32 i = 0; i < j->rows; i++) out[i] = p->output[i];
    return TS_OK;
}

i32 ts_release(struct tensor_service *s, u32 handle)
{
    struct ts_job *j = lookup(s, handle);
    if (!j) return TS_STALE;
    if (j->state != 2) return TS_INVALID;
    struct ts_payload *p = &s->payload[(handle & 7U) - 1U];
    for (u32 i = 0; i < sizeof(p->matrix); i++) p->matrix[i] = 0;
    for (u32 i = 0; i < TS_COLS; i++) p->vector[i] = 0;
    for (u32 i = 0; i < TS_ROWS; i++) p->output[i] = 0;
    j->backend = 0;
    j->backend_generation = 0;
    j->rows = j->cols = 0;
    j->status = TS_STALE;
    j->state = 0;
    return TS_OK;
}

static bool span(pv_ctx *c, i32 h, u32 *p, u32 *n)
{
    if (h < 0 || h >= c->span_count || h >= PV_MAX_SPANS ||
        c->span_len[h] < 0) return false;
    *p = c->span_ptr[h];
    *n = (u32)c->span_len[h];
    return *p <= (u32)c->mem_size && *n <= (u32)c->mem_size - *p;
}

static i32 vm_matvec(struct tensor_service *s, pv_ctx *c, u32 pin,
                     i32 a, i32 x, i32 rows, i32 cols, bool envelope)
{
    u32 ap, an, xp, xn, handle;
    u32 prefix = envelope ? 8U : 0U;
    if (!c || !c->mem || c->mem_size < 0 || (u64)c->mem_size > 0xffffffffULL ||
        rows <= 0 || rows > (i32)TS_ROWS ||
        cols <= 0 || cols > (i32)TS_COLS ||
        c->no_alloc || c->span_count < 0 || c->span_count >= PV_MAX_SPANS ||
        !span(c, a, &ap, &an) || !span(c, x, &xp, &xn) ||
        c->arena_top > (u32)c->mem_size ||
        (u32)rows * 4U + prefix > (u32)c->mem_size - c->arena_top)
        return TS_INVALID;
    i32 rc = ts_submit(s, pin, TS_I8_I32, (const i8 *)c->mem + ap, an,
        (const i8 *)c->mem + xp, xn, (u32)rows, (u32)cols, &handle);
    if (rc) return rc;
    /* Private same-core queue: at most DEPTH scheduled steps to our result. */
    for (u32 i = 0; i < TS_DEPTH && lookup(s, handle)->state != 2; i++)
        ts_step(s);
    i32 out[TS_ROWS];
    rc = ts_result(s, handle, out, TS_ROWS);
    if (!rc) {
        u32 base = c->arena_top;
        if (envelope) {
            c->mem[base] = 'T'; c->mem[base + 1] = 'S';
            c->mem[base + 2] = 'R'; c->mem[base + 3] = '1';
            c->mem[base + 4] = 0; c->mem[base + 5] = 0;
            c->mem[base + 6] = 0;
            c->mem[base + 7] = (u8)lookup(s, handle)->backend;
        }
        for (u32 i = 0; i < (u32)rows; i++)
            for (u32 b = 0; b < 4; b++)
                c->mem[base + prefix + i * 4U + b] = (u8)((u32)out[i] >> (24U - b * 8U));
        i32 h = c->span_count++;
        c->span_ptr[h] = base;
        c->span_len[h] = rows * 4 + (i32)prefix;
        c->arena_top += (u32)rows * 4U + prefix;
        rc = h;
    }
    ts_release(s, handle);
    return rc;
}

i32 ts_vm_matvec(struct tensor_service *s, pv_ctx *c, u32 pin, i32 a, i32 x)
{
    if (!c) return TS_INVALID;
    return vm_matvec(s, c, pin, a, x, c->tensor_rows, c->tensor_cols, false);
}

static u32 be32(const u8 *p)
{
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

i32 ts_vm_pinned(struct tensor_service *s, pv_ctx *c, i32 request, u32 pin)
{
    u32 p, n;
    if (!c || !c->mem || c->mem_size < 0 || (u64)c->mem_size > 0xffffffffULL ||
        !span(c, request, &p, &n) || n != 20U) return TS_INVALID;
    const u8 *r = c->mem + p;
    if (be32(r) != 0x54534d31U || pin > TS_QPU) return TS_INVALID;
    u32 rows = be32(r + 4), cols = be32(r + 8);
    u32 a = be32(r + 12), x = be32(r + 16);
    if (rows > TS_ROWS || cols > TS_COLS || a >= PV_MAX_SPANS ||
        x >= PV_MAX_SPANS) return TS_INVALID;
    return vm_matvec(s, c, pin, (i32)a, (i32)x, (i32)rows, (i32)cols, true);
}

i32 ts_vm_selected(pv_ctx *c, i32 result)
{
    u32 p, n;
    if (!c || !c->mem || c->mem_size < 0 || (u64)c->mem_size > 0xffffffffULL ||
        !span(c, result, &p, &n) || n < 12U || n > 8U + TS_ROWS * 4U ||
        (n & 3U)) return TS_INVALID;
    const u8 *r = c->mem + p;
    u32 id = be32(r + 4);
    return be32(r) == 0x54535231U && id >= TS_CPU && id <= TS_QPU
        ? (i32)id : TS_INVALID;
}
