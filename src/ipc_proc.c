#include "ipc_proc.h"
#include "principal.h"
#include "core_env.h"
#include "simd.h"

struct proc_fifo_channel {
    bool used;
    u8 name[PROC_IPC_NAME_MAX + 1];
    u32 owner_core;
    u32 owner_principal;
    u32 owner_pid;
    u32 peer_principal;
    u32 owner_acl;
    u32 peer_acl;
    u32 depth;
    u32 msg_max;
    u32 head;
    u32 count;
    u16 lens[PROC_IPC_FIFO_DEPTH_MAX];
    u8  frames[PROC_IPC_FIFO_DEPTH_MAX][PROC_IPC_FIFO_MSG_MAX];
};

struct proc_shm_region {
    bool used;
    u8 name[PROC_IPC_NAME_MAX + 1];
    u32 owner_core;
    u32 owner_principal;
    u32 owner_pid;
    u32 peer_principal;
    u32 owner_acl;
    u32 peer_acl;
    u32 size;
    u32 offset;
};

struct proc_shm_map {
    bool used;
    i32 region_id;
    u32 core;
    u32 principal;
    u32 pid;
    u32 flags;
};

static struct proc_fifo_channel g_fifos[PROC_IPC_FIFO_MAX];
static struct proc_shm_region g_regions[PROC_IPC_SHM_MAX_REGIONS];
static struct proc_shm_map g_maps[PROC_IPC_SHM_MAX_MAPS];
static u32 g_shm_pool_off;

static bool ascii_name_ok(const char *name)
{
    if (!name) return false;
    for (u32 i = 0; i <= PROC_IPC_NAME_MAX; i++) {
        u8 c = (u8)name[i];
        if (c == 0) return i != 0;
        if (c < 0x20 || c > 0x7E) return false;
    }
    return false;
}

static bool name_eq(const u8 *a, const char *b)
{
    for (u32 i = 0; i <= PROC_IPC_NAME_MAX; i++) {
        if (a[i] != (u8)b[i]) return false;
        if (a[i] == 0) return true;
    }
    return false;
}

static void copy_bytes(void *dst, const void *src, u32 n)
{
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    for (u32 i = 0; i < n; i++) d[i] = s[i];
}

static i32 fifo_find_by_name(const char *name)
{
    for (u32 i = 0; i < PROC_IPC_FIFO_MAX; i++) {
        if (!g_fifos[i].used) continue;
        if (name_eq(g_fifos[i].name, name)) return (i32)i;
    }
    return PROC_IPC_ERR_NOENT;
}

static i32 region_find_by_name(const char *name)
{
    for (u32 i = 0; i < PROC_IPC_SHM_MAX_REGIONS; i++) {
        if (!g_regions[i].used) continue;
        if (name_eq(g_regions[i].name, name)) return (i32)i;
    }
    return PROC_IPC_ERR_NOENT;
}

static u32 principal_acl(u32 principal, u32 owner_principal, u32 peer_principal,
                         u32 owner_acl, u32 peer_acl)
{
    if (principal == PRINCIPAL_ROOT) return 0xFFFFFFFFU;
    if (principal == owner_principal) return owner_acl;
    if (peer_principal == PROC_IPC_PEER_ANY || principal == peer_principal)
        return peer_acl;
    return 0;
}

void ipc_proc_init(void)
{
    memset(g_fifos, 0, sizeof(g_fifos));
    memset(g_regions, 0, sizeof(g_regions));
    memset(g_maps, 0, sizeof(g_maps));
    simd_zero((void *)(usize)IPC_SHM_BASE, IPC_SHM_SIZE);
    g_shm_pool_off = 0;
    dsb();
}

i32 ipc_proc_fifo_create(u32 owner_principal, u32 owner_pid, const char *name,
                         u32 peer_principal, u32 owner_acl, u32 peer_acl,
                         u32 depth, u32 msg_max)
{
    if (!ascii_name_ok(name)) return PROC_IPC_ERR_INVAL;
    if (peer_principal != PROC_IPC_PEER_ANY && peer_principal >= PRINCIPAL_MAX)
        return PROC_IPC_ERR_INVAL;
    if (depth == 0 || depth > PROC_IPC_FIFO_DEPTH_MAX) return PROC_IPC_ERR_INVAL;
    if (msg_max == 0 || msg_max > PROC_IPC_FIFO_MSG_MAX) return PROC_IPC_ERR_INVAL;
    if (fifo_find_by_name(name) >= 0) return PROC_IPC_ERR_EXISTS;
    if ((owner_acl & (PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV)) == 0)
        return PROC_IPC_ERR_INVAL;
    if ((peer_acl & (PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV)) == 0 &&
        peer_principal != owner_principal && peer_principal != PROC_IPC_PEER_ANY)
        return PROC_IPC_ERR_INVAL;

    for (u32 i = 0; i < PROC_IPC_FIFO_MAX; i++) {
        if (g_fifos[i].used) continue;
        struct proc_fifo_channel *ch = &g_fifos[i];
        memset(ch, 0, sizeof(*ch));
        ch->used = true;
        ch->owner_core = core_id();
        ch->owner_principal = owner_principal;
        ch->owner_pid = owner_pid;
        ch->peer_principal = peer_principal;
        ch->owner_acl = owner_acl;
        ch->peer_acl = peer_acl;
        ch->depth = depth;
        ch->msg_max = msg_max;
        copy_bytes(ch->name, name, PROC_IPC_NAME_MAX + 1);
        dmb();
        return (i32)i;
    }
    return PROC_IPC_ERR_NOSPC;
}

i32 ipc_proc_fifo_open(u32 principal, u32 pid, const char *name, u32 want_acl)
{
    (void)pid;
    i32 id = fifo_find_by_name(name);
    if (id < 0) return id;
    struct proc_fifo_channel *ch = &g_fifos[id];
    if (ch->owner_core != core_id()) return PROC_IPC_ERR_UNSUPPORTED;
    u32 acl = principal_acl(principal, ch->owner_principal, ch->peer_principal,
                            ch->owner_acl, ch->peer_acl);
    if ((want_acl & ~(PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV)) != 0)
        return PROC_IPC_ERR_INVAL;
    if ((acl & want_acl) != want_acl)
        return PROC_IPC_ERR_ACCESS;
    return id;
}

i32 ipc_proc_fifo_send(u32 principal, i32 channel_id, const void *data, u32 len)
{
    if (!data || len == 0) return PROC_IPC_ERR_INVAL;
    if (channel_id < 0 || channel_id >= PROC_IPC_FIFO_MAX) return PROC_IPC_ERR_INVAL;
    struct proc_fifo_channel *ch = &g_fifos[channel_id];
    if (!ch->used) return PROC_IPC_ERR_NOENT;
    if (ch->owner_core != core_id()) return PROC_IPC_ERR_UNSUPPORTED;
    u32 acl = principal_acl(principal, ch->owner_principal, ch->peer_principal,
                            ch->owner_acl, ch->peer_acl);
    if ((acl & PROC_IPC_PERM_SEND) == 0) return PROC_IPC_ERR_ACCESS;
    if (len > ch->msg_max) return PROC_IPC_ERR_TOOLONG;
    if (ch->count == ch->depth) return PROC_IPC_ERR_FULL;

    u32 idx = (ch->head + ch->count) % ch->depth;
    ch->lens[idx] = (u16)len;
    copy_bytes(ch->frames[idx], data, len);
    dmb(); /* payload visible before queue metadata update */
    ch->count++;
    sev();
    return PROC_IPC_OK;
}

i32 ipc_proc_fifo_recv(u32 principal, i32 channel_id, void *out, u32 out_max, u32 *len_out)
{
    if (!out || !len_out) return PROC_IPC_ERR_INVAL;
    if (channel_id < 0 || channel_id >= PROC_IPC_FIFO_MAX) return PROC_IPC_ERR_INVAL;
    struct proc_fifo_channel *ch = &g_fifos[channel_id];
    if (!ch->used) return PROC_IPC_ERR_NOENT;
    if (ch->owner_core != core_id()) return PROC_IPC_ERR_UNSUPPORTED;
    u32 acl = principal_acl(principal, ch->owner_principal, ch->peer_principal,
                            ch->owner_acl, ch->peer_acl);
    if ((acl & PROC_IPC_PERM_RECV) == 0) return PROC_IPC_ERR_ACCESS;
    if (ch->count == 0) return PROC_IPC_ERR_EMPTY;

    dmb(); /* consume queue metadata before reading payload */
    u32 idx = ch->head;
    u32 len = ch->lens[idx];
    if (out_max < len) return PROC_IPC_ERR_TOOLONG;
    copy_bytes(out, ch->frames[idx], len);
    dmb(); /* payload consumed before head/count advance */
    ch->head = (ch->head + 1) % ch->depth;
    ch->count--;
    *len_out = len;
    return PROC_IPC_OK;
}

i32 ipc_proc_shm_create(u32 owner_principal, u32 owner_pid, const char *name,
                        u32 peer_principal, u32 owner_acl, u32 peer_acl,
                        u32 size)
{
    if (!ascii_name_ok(name)) return PROC_IPC_ERR_INVAL;
    if (peer_principal != PROC_IPC_PEER_ANY && peer_principal >= PRINCIPAL_MAX)
        return PROC_IPC_ERR_INVAL;
    if (size == 0 || size > PROC_IPC_SHM_REGION_MAX) return PROC_IPC_ERR_INVAL;
    if ((owner_acl & (PROC_IPC_PERM_MAP_READ | PROC_IPC_PERM_MAP_WRITE)) == 0)
        return PROC_IPC_ERR_INVAL;
    if ((peer_acl & (PROC_IPC_PERM_MAP_READ | PROC_IPC_PERM_MAP_WRITE)) == 0 &&
        peer_principal != owner_principal && peer_principal != PROC_IPC_PEER_ANY)
        return PROC_IPC_ERR_INVAL;
    if (region_find_by_name(name) >= 0) return PROC_IPC_ERR_EXISTS;

    u32 aligned = (size + 63U) & ~63U;
    if ((g_shm_pool_off + aligned) > IPC_SHM_SIZE)
        return PROC_IPC_ERR_NOSPC;

    for (u32 i = 0; i < PROC_IPC_SHM_MAX_REGIONS; i++) {
        if (g_regions[i].used) continue;
        struct proc_shm_region *r = &g_regions[i];
        memset(r, 0, sizeof(*r));
        r->used = true;
        r->owner_core = core_id();
        r->owner_principal = owner_principal;
        r->owner_pid = owner_pid;
        r->peer_principal = peer_principal;
        r->owner_acl = owner_acl;
        r->peer_acl = peer_acl;
        r->size = size;
        r->offset = g_shm_pool_off;
        copy_bytes(r->name, name, PROC_IPC_NAME_MAX + 1);
        simd_zero((void *)(usize)(IPC_SHM_BASE + r->offset), aligned);
        g_shm_pool_off += aligned;
        dmb();
        return (i32)i;
    }
    return PROC_IPC_ERR_NOSPC;
}

i32 ipc_proc_shm_open(u32 principal, u32 pid, const char *name, u32 want_acl)
{
    (void)pid;
    i32 id = region_find_by_name(name);
    if (id < 0) return id;
    struct proc_shm_region *r = &g_regions[id];
    if (r->owner_core != core_id()) return PROC_IPC_ERR_UNSUPPORTED;
    u32 acl = principal_acl(principal, r->owner_principal, r->peer_principal,
                            r->owner_acl, r->peer_acl);
    if ((want_acl & ~(PROC_IPC_PERM_MAP_READ | PROC_IPC_PERM_MAP_WRITE)) != 0)
        return PROC_IPC_ERR_INVAL;
    if ((acl & want_acl) != want_acl)
        return PROC_IPC_ERR_ACCESS;
    return id;
}

i32 ipc_proc_shm_map(u32 principal, u32 pid, i32 region_id, u32 req_flags,
                     void **addr_out, u32 *size_out)
{
    if (!addr_out || !size_out) return PROC_IPC_ERR_INVAL;
    if (region_id < 0 || region_id >= PROC_IPC_SHM_MAX_REGIONS) return PROC_IPC_ERR_INVAL;
    if ((req_flags & (PROC_IPC_MAP_READ | PROC_IPC_MAP_WRITE)) == 0) return PROC_IPC_ERR_INVAL;
    if ((req_flags & PROC_IPC_MAP_EXEC) != 0) return PROC_IPC_ERR_UNSUPPORTED;
    if ((req_flags & ~(PROC_IPC_MAP_READ | PROC_IPC_MAP_WRITE | PROC_IPC_MAP_EXEC)) != 0)
        return PROC_IPC_ERR_INVAL;

    struct proc_shm_region *r = &g_regions[region_id];
    if (!r->used) return PROC_IPC_ERR_NOENT;
    if (r->owner_core != core_id()) return PROC_IPC_ERR_UNSUPPORTED;

    u32 acl = principal_acl(principal, r->owner_principal, r->peer_principal,
                            r->owner_acl, r->peer_acl);
    u32 needed = 0;
    if (req_flags & PROC_IPC_MAP_READ) needed |= PROC_IPC_PERM_MAP_READ;
    if (req_flags & PROC_IPC_MAP_WRITE) needed |= PROC_IPC_PERM_MAP_WRITE;
    if ((acl & needed) != needed) return PROC_IPC_ERR_ACCESS;

    for (u32 i = 0; i < PROC_IPC_SHM_MAX_MAPS; i++) {
        if (g_maps[i].used) continue;
        g_maps[i].used = true;
        g_maps[i].region_id = region_id;
        g_maps[i].core = core_id();
        g_maps[i].principal = principal;
        g_maps[i].pid = pid;
        g_maps[i].flags = req_flags;
        dmb(); /* map metadata committed before address handoff */
        *addr_out = (void *)(usize)(IPC_SHM_BASE + r->offset);
        *size_out = r->size;
        dmb(); /* address/size visible before caller consumes handle */
        return (i32)i;
    }
    return PROC_IPC_ERR_NOSPC;
}

i32 ipc_proc_shm_unmap(u32 principal, u32 pid, i32 map_handle)
{
    if (map_handle < 0 || map_handle >= PROC_IPC_SHM_MAX_MAPS) return PROC_IPC_ERR_INVAL;
    struct proc_shm_map *m = &g_maps[map_handle];
    if (!m->used) return PROC_IPC_ERR_NOENT;
    if (m->core != core_id()) return PROC_IPC_ERR_UNSUPPORTED;
    if (m->principal != principal || m->pid != pid)
        return PROC_IPC_ERR_ACCESS;
    dmb(); /* caller must complete writes before releasing map handle */
    memset(m, 0, sizeof(*m));
    dmb();
    return PROC_IPC_OK;
}
