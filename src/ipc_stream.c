#include "ipc_stream.h"
#include "kspin.h"

struct topic_sub {
    bool used;
    u32 generation;
    u32 cursor_seq;
    u32 _pad[13];
} ALIGNED(64);

struct ipc_topic_obj {
    bool used;
    u32 generation;
    u8 name[IPC_NAME_MAX + 1];
    u32 flags;
    u32 window;
    u32 event_max;
    u32 head;
    u32 count;
    u32 next_seq;
    u16 lens[IPC_TOPIC_WINDOW_MAX];
    u32 seqs[IPC_TOPIC_WINDOW_MAX];
    u8 events[IPC_TOPIC_WINDOW_MAX][IPC_TOPIC_EVENT_MAX];
    struct topic_sub subs[IPC_TOPIC_SUBS_MAX];
} ALIGNED(64);

static struct ipc_topic_obj g_topics[IPC_TOPIC_MAX];
static bool g_persist_runtime;
static struct kspinlock g_topic_lock;
static const char *const g_topic_walfs_path = "/var/ipc/topics";
_Static_assert(sizeof(struct topic_sub) == 64,
               "IPC topic subscriptions must be one cache line");
_Static_assert((sizeof(struct ipc_topic_obj) & 63U) == 0U,
               "IPC topic objects must have cache-line stride");

#define IPC_TOPIC_POISON_U32 0xDEAD7070U

static u32 topic_bump_generation(u32 old)
{
    u32 g = old + 1U;
    return g ? g : 1U;
}

static void topic_poison(struct ipc_topic_obj *t)
{
    if (!t)
        return;
    u32 gen = topic_bump_generation(t->generation);
    u32 sub_gen[IPC_TOPIC_SUBS_MAX];
    for (u32 i = 0; i < IPC_TOPIC_SUBS_MAX; i++)
        sub_gen[i] = topic_bump_generation(t->subs[i].generation);
    memset(t, 0xA5, sizeof(*t));
    t->used = false;
    t->generation = gen;
    t->flags = IPC_TOPIC_POISON_U32;
    t->window = 0;
    t->event_max = 0;
    t->head = IPC_TOPIC_POISON_U32;
    t->count = IPC_TOPIC_POISON_U32;
    t->next_seq = IPC_TOPIC_POISON_U32;
    for (u32 i = 0; i < IPC_TOPIC_SUBS_MAX; i++) {
        t->subs[i].used = false;
        t->subs[i].generation = sub_gen[i];
        t->subs[i].cursor_seq = IPC_TOPIC_POISON_U32;
    }
}

static void copy_bytes(void *dst, const void *src, u32 n)
{
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    for (u32 i = 0; i < n; i++) d[i] = s[i];
}

static bool ascii_name_ok(const char *name)
{
    if (!name) return false;
    for (u32 i = 0; i <= IPC_NAME_MAX; i++) {
        u8 c = (u8)name[i];
        if (c == 0) return i != 0;
        if (c < 0x20 || c > 0x7E) return false;
    }
    return false;
}

static i32 topic_find(const char *name)
{
    for (u32 i = 0; i < IPC_TOPIC_MAX; i++) {
        if (!g_topics[i].used) continue;
        u32 j = 0;
        while (j <= IPC_NAME_MAX) {
            if (g_topics[i].name[j] != (u8)name[j]) break;
            if (name[j] == 0) return (i32)i;
            j++;
        }
    }
    return IPC_ERR_NOENT;
}

static struct ipc_topic_obj *topic_from_handle(i32 handle)
{
    if (handle < 0 || handle >= IPC_TOPIC_MAX) return NULL;
    if (!g_topics[handle].used) return NULL;
    return &g_topics[handle];
}

static i32 sub_encode(u32 topic, u32 sub)
{
    return (i32)(((topic & 0xFFU) << 8) | ((sub + 1U) & 0xFFU));
}

static bool sub_decode(i32 handle, u32 *topic_out, u32 *sub_out)
{
    if (handle < 0) return false;
    u32 raw = (u32)handle;
    u32 topic = (raw >> 8) & 0xFFU;
    u32 sub1 = raw & 0xFFU;
    if (sub1 == 0) return false;
    u32 sub = sub1 - 1U;
    if (topic >= IPC_TOPIC_MAX || sub >= IPC_TOPIC_SUBS_MAX) return false;
    if (topic_out) *topic_out = topic;
    if (sub_out) *sub_out = sub;
    return true;
}

static u32 topic_tail(const struct ipc_topic_obj *t)
{
    return (t->head + t->count) % t->window;
}

void ipc_stream_init(void)
{
    kspin_lock(&g_topic_lock);
    for (u32 i = 0; i < IPC_TOPIC_MAX; i++)
        topic_poison(&g_topics[i]);
    g_persist_runtime = false;
    kspin_unlock(&g_topic_lock);
}

void ipc_stream_set_persistence(bool enabled)
{
    kspin_lock(&g_topic_lock);
    g_persist_runtime = enabled;
    kspin_unlock(&g_topic_lock);
}

static i32 ipc_topic_create_unlocked(const char *name, u32 replay_window, u32 flags, u32 event_max)
{
    if (!ascii_name_ok(name)) return IPC_ERR_INVAL;
    if (replay_window == 0 || replay_window > IPC_TOPIC_WINDOW_MAX) return IPC_ERR_INVAL;
    if (event_max == 0 || event_max > IPC_TOPIC_EVENT_MAX) return IPC_ERR_INVAL;
    if (topic_find(name) >= 0) return IPC_ERR_EXISTS;

    for (u32 i = 0; i < IPC_TOPIC_MAX; i++) {
        if (g_topics[i].used) continue;
        struct ipc_topic_obj *t = &g_topics[i];
        u32 gen = topic_bump_generation(t->generation);
        memset(t, 0, sizeof(*t));
        t->used = true;
        t->generation = gen;
        t->flags = flags;
        t->window = replay_window;
        t->event_max = event_max;
        t->next_seq = 1;
        copy_bytes(t->name, name, IPC_NAME_MAX + 1);
        return (i32)i;
    }
    return IPC_ERR_NOSPC;
}

static i32 ipc_topic_open_unlocked(const char *name)
{
    if (!ascii_name_ok(name)) return IPC_ERR_INVAL;
    return topic_find(name);
}

static i32 ipc_topic_publish_unlocked(i32 topic_handle, const void *data, u32 len)
{
    struct ipc_topic_obj *t = topic_from_handle(topic_handle);
    if (!t || !data) return IPC_ERR_INVAL;
    if (len == 0 || len > t->event_max) return IPC_ERR_TOOLONG;

    if (t->count == t->window) {
        u32 idx = topic_tail(t);
        t->lens[idx] = (u16)len;
        t->seqs[idx] = t->next_seq++;
        copy_bytes(t->events[idx], data, len);
        t->head = (t->head + 1) % t->window;
        return IPC_OK;
    }

    u32 idx = topic_tail(t);
    t->lens[idx] = (u16)len;
    t->seqs[idx] = t->next_seq++;
    copy_bytes(t->events[idx], data, len);
    t->count++;
    return IPC_OK;
}

static i32 ipc_topic_subscribe_unlocked(i32 topic_handle)
{
    struct ipc_topic_obj *t = topic_from_handle(topic_handle);
    if (!t) return IPC_ERR_INVAL;

    for (u32 i = 0; i < IPC_TOPIC_SUBS_MAX; i++) {
        if (t->subs[i].used) continue;
        u32 gen = topic_bump_generation(t->subs[i].generation);
        memset(&t->subs[i], 0, sizeof(t->subs[i]));
        t->subs[i].used = true;
        t->subs[i].generation = gen;
        t->subs[i].cursor_seq = (t->count == 0) ? t->next_seq : (t->next_seq - t->count);
        return sub_encode((u32)topic_handle, i);
    }
    return IPC_ERR_NOSPC;
}

static i32 ipc_topic_read_unlocked(i32 sub_handle, void *out, u32 out_max, u32 *len_out)
{
    u32 topic_idx, sub_idx;
    if (!sub_decode(sub_handle, &topic_idx, &sub_idx)) return IPC_ERR_INVAL;
    struct ipc_topic_obj *t = topic_from_handle((i32)topic_idx);
    if (!t || !out) return IPC_ERR_INVAL;
    if (!t->subs[sub_idx].used) return IPC_ERR_INVAL;
    if (t->count == 0) return IPC_ERR_EMPTY;

    u32 oldest_seq = t->next_seq - t->count;
    if (t->subs[sub_idx].cursor_seq < oldest_seq)
        t->subs[sub_idx].cursor_seq = oldest_seq;
    if (t->subs[sub_idx].cursor_seq >= t->next_seq)
        return IPC_ERR_EMPTY;

    u32 rel = t->subs[sub_idx].cursor_seq - oldest_seq;
    u32 slot = (t->head + rel) % t->window;
    u32 len = t->lens[slot];
    if (out_max < len) return IPC_ERR_TOOLONG;

    copy_bytes(out, t->events[slot], len);
    if (len_out) *len_out = len;
    t->subs[sub_idx].cursor_seq++;
    return IPC_OK;
}

static i32 ipc_topic_flush_unlocked(i32 topic_handle)
{
    struct ipc_topic_obj *t = topic_from_handle(topic_handle);
    if (!t) return IPC_ERR_INVAL;
    if ((t->flags & IPC_TOPIC_F_PERSIST) == 0)
        return IPC_OK;

    if (!g_persist_runtime)
        return IPC_ERR_UNSUPPORTED;

#if IPC_TOPIC_PERSIST_WALFS
    (void)g_topic_walfs_path;
    return IPC_ERR_UNSUPPORTED;
#else
    (void)g_topic_walfs_path;
    return IPC_ERR_UNSUPPORTED;
#endif
}

static i32 ipc_topic_len_unlocked(i32 topic_handle)
{
    struct ipc_topic_obj *t = topic_from_handle(topic_handle);
    if (!t) return IPC_ERR_INVAL;
    return (i32)t->count;
}

i32 ipc_topic_create(const char *name, u32 replay_window, u32 flags, u32 event_max)
{
    kspin_lock(&g_topic_lock);
    i32 rc = ipc_topic_create_unlocked(name, replay_window, flags, event_max);
    kspin_unlock(&g_topic_lock);
    return rc;
}

i32 ipc_topic_open(const char *name)
{
    kspin_lock(&g_topic_lock);
    i32 rc = ipc_topic_open_unlocked(name);
    kspin_unlock(&g_topic_lock);
    return rc;
}

i32 ipc_topic_publish(i32 topic_handle, const void *data, u32 len)
{
    kspin_lock(&g_topic_lock);
    i32 rc = ipc_topic_publish_unlocked(topic_handle, data, len);
    kspin_unlock(&g_topic_lock);
    return rc;
}

i32 ipc_topic_subscribe(i32 topic_handle)
{
    kspin_lock(&g_topic_lock);
    i32 rc = ipc_topic_subscribe_unlocked(topic_handle);
    kspin_unlock(&g_topic_lock);
    return rc;
}

i32 ipc_topic_read(i32 sub_handle, void *out, u32 out_max, u32 *len_out)
{
    kspin_lock(&g_topic_lock);
    i32 rc = ipc_topic_read_unlocked(sub_handle, out, out_max, len_out);
    kspin_unlock(&g_topic_lock);
    return rc;
}

i32 ipc_topic_flush(i32 topic_handle)
{
    kspin_lock(&g_topic_lock);
    i32 rc = ipc_topic_flush_unlocked(topic_handle);
    kspin_unlock(&g_topic_lock);
    return rc;
}

i32 ipc_topic_len(i32 topic_handle)
{
    kspin_lock(&g_topic_lock);
    i32 rc = ipc_topic_len_unlocked(topic_handle);
    kspin_unlock(&g_topic_lock);
    return rc;
}
