#ifndef PICOWAL_INDEX_CONTEXT_H
#define PICOWAL_INDEX_CONTEXT_H

#include <stdint.h>

typedef struct pw_pack_index_ctx {
    uint32_t pack_id;
    uint64_t last_used;
    uint8_t used;
    uint8_t dirty;
    uint8_t pinned;
    void *state;
} pw_pack_index_ctx;

typedef int (*pw_index_context_persist_fn)(pw_pack_index_ctx *context, void *user);
typedef int (*pw_index_context_load_fn)(pw_pack_index_ctx *context, uint32_t pack_id, void *user);

typedef struct {
    pw_pack_index_ctx *contexts;
    uint32_t capacity;
    uint64_t clock;
    pw_index_context_persist_fn persist;
    pw_index_context_load_fn load;
    void *user;
} pw_index_context_table;

void pw_index_context_init(pw_index_context_table *table,
                           pw_pack_index_ctx *contexts, uint32_t capacity,
                           pw_index_context_persist_fn persist,
                           pw_index_context_load_fn load, void *user);
pw_pack_index_ctx *pw_index_context_acquire(pw_index_context_table *table,
                                             uint32_t pack_id);
int pw_index_context_release(pw_index_context_table *table, uint32_t pack_id);
int pw_index_context_mark_dirty(pw_index_context_table *table, uint32_t pack_id);

#endif
