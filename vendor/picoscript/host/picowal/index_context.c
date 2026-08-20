#include "index_context.h"
#include <string.h>

void pw_index_context_init(pw_index_context_table *t, pw_pack_index_ctx *c,
                           uint32_t n, pw_index_context_persist_fn persist,
                           pw_index_context_load_fn load, void *user)
{
    memset(t, 0, sizeof(*t)); t->contexts = c; t->capacity = n;
    t->persist = persist; t->load = load; t->user = user;
    memset(c, 0, (size_t)n * sizeof(*c));
}

static pw_pack_index_ctx *find(pw_index_context_table *t, uint32_t pack)
{
    uint32_t i; for (i = 0; i < t->capacity; ++i)
        if (t->contexts[i].used && t->contexts[i].pack_id == pack) return &t->contexts[i];
    return NULL;
}

pw_pack_index_ctx *pw_index_context_acquire(pw_index_context_table *t, uint32_t pack)
{
    uint32_t i; pw_pack_index_ctx *victim = NULL, *free_slot = NULL, *ctx = find(t, pack);
    ++t->clock;
    if (ctx) { ctx->last_used = t->clock; return ctx; }
    for (i = 0; i < t->capacity; ++i) {
        if (!t->contexts[i].used) { free_slot = &t->contexts[i]; break; }
        if (!t->contexts[i].pinned && (!victim || t->contexts[i].last_used < victim->last_used ||
            (t->contexts[i].last_used == victim->last_used && t->contexts[i].pack_id < victim->pack_id)))
            victim = &t->contexts[i];
    }
    ctx = free_slot ? free_slot : victim;
    if (!ctx || (ctx == victim && ctx->dirty && (!t->persist || t->persist(ctx, t->user) != 0))) return NULL;
    memset(ctx, 0, sizeof(*ctx)); ctx->used = 1; ctx->pack_id = pack; ctx->last_used = t->clock;
    if (t->load && t->load(ctx, pack, t->user) != 0) { memset(ctx, 0, sizeof(*ctx)); return NULL; }
    return ctx;
}

int pw_index_context_release(pw_index_context_table *t, uint32_t pack)
{
    pw_pack_index_ctx *ctx = find(t, pack); if (!ctx) return -1;
    if (ctx->dirty && (!t->persist || t->persist(ctx, t->user) != 0)) return -1;
    memset(ctx, 0, sizeof(*ctx)); return 0;
}

int pw_index_context_mark_dirty(pw_index_context_table *t, uint32_t pack)
{ pw_pack_index_ctx *ctx = find(t, pack); if (!ctx) return -1; ctx->dirty = 1; return 0; }
