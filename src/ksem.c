#include "ksem.h"
#include "core.h"

struct ksem_obj {
    bool used;
    i32 count;
    i32 max_count;
};

static struct ksem_obj sem_table[NUM_CORES][KSEM_MAX_PER_CORE];

static inline u32 sem_owner(i32 sem_id)
{
    return ((u32)sem_id >> 8) & 0xFFU;
}

static inline u32 sem_slot(i32 sem_id)
{
    return (u32)sem_id & 0xFFU;
}

static struct ksem_obj *ksem_get_local(i32 sem_id)
{
    u32 owner = sem_owner(sem_id);
    u32 slot = sem_slot(sem_id);
    if (owner != core_id() || owner >= NUM_CORES || slot >= KSEM_MAX_PER_CORE)
        return NULL;
    struct ksem_obj *s = &sem_table[owner][slot];
    return s->used ? s : NULL;
}

void ksem_init_core(void)
{
    u32 owner = core_id() & 3U;
    for (u32 i = 0; i < KSEM_MAX_PER_CORE; i++) {
        sem_table[owner][i].used = false;
        sem_table[owner][i].count = 0;
        sem_table[owner][i].max_count = 0;
    }
}

i32 ksem_create(u32 initial, u32 max_count)
{
    u32 owner = core_id() & 3U;
    if (max_count == 0 || initial > max_count)
        return KSEM_ERR;

    for (u32 i = 0; i < KSEM_MAX_PER_CORE; i++) {
        struct ksem_obj *s = &sem_table[owner][i];
        if (!s->used) {
            s->used = true;
            s->count = (i32)initial;
            s->max_count = (i32)max_count;
            dmb();
            return (i32)KSEM_ID(owner, i);
        }
    }
    return KSEM_ERR;
}

i32 ksem_trywait(i32 sem_id)
{
    struct ksem_obj *s = ksem_get_local(sem_id);
    if (!s)
        return KSEM_ERR;

    if (s->count <= 0)
        return KSEM_WOULD_BLOCK;

    s->count--;
    dmb();
    return KSEM_OK;
}

i32 ksem_wait(i32 sem_id)
{
    for (;;) {
        i32 r = ksem_trywait(sem_id);
        if (r == KSEM_OK || r == KSEM_ERR)
            return r;
        wfe();
    }
}

i32 ksem_post(i32 sem_id)
{
    struct ksem_obj *s = ksem_get_local(sem_id);
    if (!s)
        return KSEM_ERR;

    if (s->count >= s->max_count)
        return KSEM_ERR_FULL;

    s->count++;
    dmb();
    sev();
    return KSEM_OK;
}
