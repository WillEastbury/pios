#include "pios_exec_sched.h"
#include "pcie_fabric.h"
#include "pico_hooks.h"
#include "picovm.h"

static struct pios_exec_sched *vm_sched;

static u32 slot_of(u32 handle)
{
    u32 slot = handle & 0xFFU;
    return slot && slot <= PIOS_EXEC_JOBS ? slot - 1U : PIOS_EXEC_JOBS;
}

static struct pios_exec_job *lookup(struct pios_exec_sched *s, u32 handle)
{
    u32 slot = slot_of(handle);
    if (!s || slot >= PIOS_EXEC_JOBS) return NULL;
    struct pios_exec_job *job = &s->jobs[slot];
    return job->state && job->generation == (handle >> 8) ? job : NULL;
}

static u32 pack_reply(const struct pios_exec_sched *s)
{
    return (s->last_reused << 25) | (s->last_dma << 24) |
           ((s->last_device & 0xFFU) << 16) |
           ((s->last_backend & 0xFFU) << 8) |
           (s->last_status < 0 ? 0U : PIOS_EXEC_COMPLETE);
}

void pios_exec_init(struct pios_exec_sched *s)
{
    u32 i;
    if (!s) return;
    for (i = 0; i < sizeof(*s); i++) ((u8 *)s)[i] = 0;
}

void pios_exec_bind_cpu(struct pios_exec_sched *s, pios_exec_cpu_fn fn)
{
    if (s) s->cpu = fn;
}

void pios_exec_set_model(struct pios_exec_sched *s, u64 model_id)
{
    if (!s) return;
    s->model_id = model_id;
    if (s->model_generation != 0xFFFFFFFFU) s->model_generation++;
}

bool pios_exec_observe(struct pios_exec_sched *s,
                       const struct pcie_fabric_snapshot *fabric)
{
    u32 i;
    if (!s || !fabric) return false;
    if (s->count) return false;
    for (i = 0; i < PIOS_EXEC_JOBS; i++)
        if (s->jobs[i].state) return false;
    s->device_count = 0;
    for (i = 0; i < fabric->function_count && i < PCIE_FABRIC_MAX_FUNCTIONS; i++) {
        const struct pcie_fabric_function *fn = &fabric->functions[i];
        struct pios_exec_device *dev;
        u8 backend;
        if (!fn->reachable) continue;
        if (pcie_fabric_is_b50(fn)) backend = PIOS_EXEC_BACKEND_B50;
        else if (pcie_fabric_is_b60(fn)) backend = PIOS_EXEC_BACKEND_B60;
        else continue;
        if (s->device_count == PIOS_EXEC_DEVICES) break;
        dev = &s->devices[s->device_count++];
        dev->vendor = fn->vendor;
        dev->device_id = fn->device;
        dev->bus = fn->bus;
        dev->dev = fn->dev;
        dev->func = fn->func;
        dev->backend = backend;
        if (dev->generation != 0xFFFFFFFFU) dev->generation++;
    }
    return true;
}

static int find_device(const struct pios_exec_sched *s, u8 bus, u8 dev, u8 func)
{
    u32 i;
    for (i = 0; i < s->device_count; i++)
        if (s->devices[i].bus == bus && s->devices[i].dev == dev &&
            s->devices[i].func == func)
            return (int)i;
    return -1;
}

bool pios_exec_note_resident(struct pios_exec_sched *s, u64 model_id,
                             u32 layer, u32 expert, u32 shard,
                             u8 bus, u8 dev, u8 func, u32 bytes)
{
    u32 i, free_slot = PIOS_EXEC_RESIDENT;
    if (!s || !bytes || find_device(s, bus, dev, func) < 0) return false;
    for (i = 0; i < PIOS_EXEC_RESIDENT; i++) {
        struct pios_exec_resident *row = &s->resident[i];
        if (row->live && row->model_id == model_id && row->layer == layer &&
            row->expert == expert && row->shard == shard &&
            row->bus == bus && row->dev == dev && row->func == func) {
            row->bytes = bytes;
            return true;
        }
        if (!row->live && free_slot == PIOS_EXEC_RESIDENT) free_slot = i;
    }
    if (free_slot == PIOS_EXEC_RESIDENT) return false;
    s->resident[free_slot] = (struct pios_exec_resident){
        .model_id = model_id, .layer = layer, .expert = expert, .shard = shard,
        .bus = bus, .dev = dev, .func = func, .live = 1, .bytes = bytes,
    };
    return true;
}

static int resident_owner(const struct pios_exec_sched *s, u32 layer,
                          u32 expert, u32 shard)
{
    u32 i;
    for (i = 0; i < PIOS_EXEC_RESIDENT; i++) {
        const struct pios_exec_resident *row = &s->resident[i];
        int dev;
        if (!row->live || row->model_id != s->model_id || row->layer != layer ||
            row->expert != expert || row->shard != shard)
            continue;
        dev = find_device(s, row->bus, row->dev, row->func);
        if (dev >= 0) return dev;
    }
    return -1;
}

i32 pios_exec_where(const struct pios_exec_sched *s, u32 layer, u32 expert)
{
    u32 i;
    if (!s) return PIOS_EXEC_INVALID;
    for (i = 0; i < PIOS_EXEC_RESIDENT; i++) {
        const struct pios_exec_resident *row = &s->resident[i];
        int dev;
        if (!row->live || row->model_id != s->model_id ||
            row->layer != layer || row->expert != expert)
            continue;
        dev = find_device(s, row->bus, row->dev, row->func);
        if (dev >= 0) return dev + 1;
    }
    return PIOS_EXEC_UNAVAILABLE;
}

static i32 enqueue(struct pios_exec_sched *s, u32 layer, u32 expert, u32 shard,
                   const i8 *matrix, const i8 *vector, u32 rows, u32 cols,
                   i32 *out, u32 *handle, u32 *slot_out)
{
    u32 slot;
    struct pios_exec_job *job;
    if (!s || !handle) return PIOS_EXEC_INVALID;
    if (s->count == PIOS_EXEC_JOBS) return PIOS_EXEC_FULL;
    for (slot = 0; slot < PIOS_EXEC_JOBS; slot++)
        if (!s->jobs[slot].state && s->jobs[slot].generation != 0x00FFFFFFU)
            break;
    if (slot == PIOS_EXEC_JOBS) return PIOS_EXEC_FULL;
    job = &s->jobs[slot];
    job->generation = (job->generation + 1U) & 0x00FFFFFFU;
    if (!job->generation) job->generation = 1U;
    job->state = PIOS_EXEC_QUEUED;
    job->backend = PIOS_EXEC_BACKEND_NONE;
    job->device_index = 0;
    job->src_device = 0;
    job->dma = 0;
    job->reused = 0;
    job->layer = layer;
    job->expert = expert;
    job->shard = shard;
    job->bytes = 0;
    job->rows = rows;
    job->cols = cols;
    job->status = PIOS_EXEC_OK;
    job->model_id = s->model_id;
    job->matrix = matrix;
    job->vector = vector;
    job->out = out;
    s->ring[(s->head + s->count) % PIOS_EXEC_JOBS] = slot;
    s->count++;
    *handle = (job->generation << 8) | (slot + 1U);
    if (slot_out) *slot_out = slot;
    return PIOS_EXEC_OK;
}

i32 pios_exec_submit(struct pios_exec_sched *s, u32 layer, u32 expert,
                     u32 shard, u32 target, u32 *handle)
{
    u32 slot = 0;
    i32 rc = enqueue(s, layer, expert, shard, NULL, NULL, 0, 0, NULL, handle, &slot);
    if (rc == PIOS_EXEC_OK) s->jobs[slot].device_index = target;
    return rc;
}

i32 pios_exec_submit_cpu(struct pios_exec_sched *s, u32 layer, u32 expert,
                         u32 shard, u32 target,
                         const i8 *matrix, const i8 *vector,
                         u32 rows, u32 cols, i32 *out, u32 *handle)
{
    u32 slot = 0;
    i32 rc;
    if ((matrix || vector || out) && (!matrix || !vector || !out || !rows || !cols))
        return PIOS_EXEC_INVALID;
    rc = enqueue(s, layer, expert, shard, matrix, vector, rows, cols, out, handle, &slot);
    if (rc == PIOS_EXEC_OK) s->jobs[slot].device_index = target;
    return rc;
}

static void finish(struct pios_exec_sched *s, struct pios_exec_job *job,
                   u32 backend, u32 device, u32 src, u32 dma, u32 reused, i32 status)
{
    (void)s;
    job->backend = backend;
    job->device_index = device;
    job->src_device = src;
    job->dma = dma;
    job->reused = reused;
    job->status = status;
    job->state = status == PIOS_EXEC_OK && dma ? PIOS_EXEC_WAIT_DMA :
                 status == PIOS_EXEC_OK ? PIOS_EXEC_COMPLETE : PIOS_EXEC_FAILED;
}

static void place(struct pios_exec_sched *s, struct pios_exec_job *job)
{
    u32 target = job->device_index;
    int owner = resident_owner(s, job->layer, job->expert, job->shard);
    u32 bytes = 0;
    if (owner >= 0) {
        u32 i;
        for (i = 0; i < PIOS_EXEC_RESIDENT; i++) {
            const struct pios_exec_resident *row = &s->resident[i];
            if (row->live && row->model_id == job->model_id &&
                row->layer == job->layer && row->expert == job->expert &&
                row->shard == job->shard &&
                find_device(s, row->bus, row->dev, row->func) == owner) {
                bytes = row->bytes;
                break;
            }
        }
    }
    if (target == 0U) {
        if (owner >= 0) {
            finish(s, job, s->devices[owner].backend, (u32)owner + 1U, 0, 0, 1, PIOS_EXEC_OK);
            job->bytes = bytes;
            return;
        }
        if (s->cpu) {
            if (job->matrix && !s->cpu(job->matrix, job->vector, job->rows, job->cols, job->out))
                finish(s, job, PIOS_EXEC_BACKEND_CPU, 0, 0, 0, 0, PIOS_EXEC_UNAVAILABLE);
            else
                finish(s, job, PIOS_EXEC_BACKEND_CPU, 0, 0, 0, 0, PIOS_EXEC_OK);
            return;
        }
        finish(s, job, PIOS_EXEC_BACKEND_NONE, 0, 0, 0, 0, PIOS_EXEC_UNAVAILABLE);
        return;
    }
    if (target > s->device_count) {
        finish(s, job, PIOS_EXEC_BACKEND_NONE, 0, 0, 0, 0, PIOS_EXEC_INVALID);
        return;
    }
    if (owner >= 0 && (u32)owner + 1U == target) {
        finish(s, job, s->devices[owner].backend, target, 0, 0, 1, PIOS_EXEC_OK);
        job->bytes = bytes;
        return;
    }
    if (owner >= 0) {
        finish(s, job, s->devices[target - 1U].backend, target,
               (u32)owner + 1U, 1, 0, PIOS_EXEC_OK);
        job->bytes = bytes;
        return;
    }
    finish(s, job, PIOS_EXEC_BACKEND_NONE, 0, 0, 0, 0, PIOS_EXEC_UNAVAILABLE);
}

bool pios_exec_step(struct pios_exec_sched *s)
{
    u32 i;
    if (!s || !s->count) return false;
    for (i = 0; i < s->count; i++) {
        struct pios_exec_job *job = &s->jobs[s->ring[(s->head + i) % PIOS_EXEC_JOBS]];
        if (job->state != PIOS_EXEC_QUEUED) continue;
        place(s, job);
        return true;
    }
    return false;
}

i32 pios_exec_dma_done(struct pios_exec_sched *s, u32 handle)
{
    struct pios_exec_job *job = lookup(s, handle);
    if (!job) return PIOS_EXEC_STALE;
    if (job->state != PIOS_EXEC_WAIT_DMA) return PIOS_EXEC_BUSY;
    job->state = PIOS_EXEC_COMPLETE;
    return PIOS_EXEC_OK;
}

i32 pios_exec_reply(struct pios_exec_sched *s, u32 *packed)
{
    struct pios_exec_job *job;
    if (!s || !packed) return PIOS_EXEC_INVALID;
    if (!s->count) return PIOS_EXEC_INVALID;
    job = &s->jobs[s->ring[s->head]];
    if (job->state != PIOS_EXEC_COMPLETE && job->state != PIOS_EXEC_FAILED)
        return PIOS_EXEC_BUSY;
    s->last_backend = job->backend;
    s->last_device = job->device_index;
    s->last_dma = job->dma;
    s->last_reused = job->reused;
    s->last_status = job->status;
    *packed = job->status == PIOS_EXEC_OK ? pack_reply(s) : 0U;
    job->state = PIOS_EXEC_FREE;
    job->matrix = NULL;
    job->vector = NULL;
    job->out = NULL;
    s->head = (s->head + 1U) % PIOS_EXEC_JOBS;
    s->count--;
    return job->status;
}

void pios_exec_sched_bind(struct pios_exec_sched *s)
{
    vm_sched = s;
}

int pios_exec_sched_vm_hook(struct pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    u32 handle = 0, packed = 0;
    i32 rc;
    if (!ctx || rd < 0 || rd >= PV_NUM_REGS || rs1 < 0 || rs1 >= PV_NUM_REGS ||
        rs2 < 0 || rs2 >= PV_NUM_REGS)
        return 0;
    if (hook < PV_HOOK_EXECSCHED_SUBMIT || hook > PV_HOOK_EXECSCHED_REUSE)
        return 0;
    if (!vm_sched) {
        ctx->regs[rd] = PIOS_EXEC_UNAVAILABLE;
        return 1;
    }
    if (hook == PV_HOOK_EXECSCHED_SUBMIT) {
        u32 packed_le = (u32)ctx->regs[rs1];
        u32 packed_target = (u32)ctx->regs[rs2];
        rc = pios_exec_submit(vm_sched, packed_le >> 16, packed_le & 0xFFFFU,
                              packed_target >> 16, packed_target & 0xFFFFU, &handle);
        ctx->regs[rd] = rc == PIOS_EXEC_OK ? (i32)handle : rc;
        if (rc == PIOS_EXEC_OK) (void)pios_exec_step(vm_sched);
        return 1;
    }
    if (hook == PV_HOOK_EXECSCHED_REPLY) {
        rc = pios_exec_reply(vm_sched, &packed);
        ctx->regs[rd] = rc == PIOS_EXEC_OK ? (i32)packed : rc;
        return 1;
    }
    if (hook == PV_HOOK_EXECSCHED_WHERE) {
        ctx->regs[rd] = pios_exec_where(vm_sched, (u32)ctx->regs[rs1],
                                        (u32)ctx->regs[rs2]);
        return 1;
    }
    ctx->regs[rd] = (i32)vm_sched->last_reused;
    return 1;
}

void pios_exec_sched_install(void)
{
    pv_execsched_hook = pios_exec_sched_vm_hook;
}
