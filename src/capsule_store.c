#include "capsule_store.h"
#include "walfs.h"
#include "simd.h"

#define CAPSULE_BASE_DIR "/var/capsules"

static u32 c_strlen(const char *s)
{
    u32 n = 0;
    while (s && s[n]) n++;
    return n;
}

static u32 c_u32_dec(char *out, u32 v)
{
    char tmp[12];
    u32 n = 0;
    if (v == 0) {
        out[0] = '0';
        out[1] = 0;
        return 1;
    }
    while (v && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    for (u32 i = 0; i < n; i++)
        out[i] = tmp[n - 1U - i];
    out[n] = 0;
    return n;
}

static bool c_copy(char *dst, u32 max, const char *src)
{
    if (!dst || max == 0 || !src) return false;
    u32 i = 0;
    while (src[i]) {
        if (i + 1U >= max) return false;
        dst[i] = src[i];
        i++;
    }
    dst[i] = 0;
    return true;
}

static void c_err(char *err, u32 max, const char *msg)
{
    if (!err || max == 0) return;
    (void)c_copy(err, max, msg ? msg : "error");
}

u32 capsule_source_for(u32 program)
{
    return CAPSULE_SOURCE_BASE + program;
}

u32 capsule_code_for(u32 program)
{
    return CAPSULE_CODE_BASE + program;
}

bool capsule_pack_valid(u32 pack)
{
    return pack >= CAPSULE_PACK_MIN && pack <= CAPSULE_PACK_MAX;
}

const char *capsule_card_role(u32 card)
{
    if (card == 0) return "manifest";
    if (card >= 1U && card <= 1000U) return "exec";
    if (card >= 1001U && card <= 10000U) return "source";
    if (card >= 20000U) return "ipc";
    if (card >= 10001U) return "bytecode";
    return "card";
}

static bool ensure_dir(const char *path)
{
    if (!path || path[0] != '/') return false;
    if (walfs_find(path)) return true;
    if (path[1] == 0) return true;
    char cur[128];
    u32 p = 0;
    cur[p++] = '/';
    cur[p] = 0;
    u64 parent = WALFS_ROOT_INODE;
    const char *s = path + 1;
    while (*s) {
        char seg[64];
        u32 n = 0;
        while (*s && *s != '/') {
            if (n + 1U >= sizeof(seg)) return false;
            seg[n++] = *s++;
        }
        seg[n] = 0;
        if (*s == '/') s++;
        if (n == 0) return false;
        if (p > 1) {
            if (p + 1U >= sizeof(cur)) return false;
            cur[p++] = '/';
        }
        if (p + n + 1U >= sizeof(cur)) return false;
        for (u32 i = 0; i < n; i++) cur[p++] = seg[i];
        cur[p] = 0;
        u64 id = walfs_find(cur);
        if (!id) {
            id = walfs_create(parent, seg, WALFS_DIR, 0755);
            if (!id) return false;
        } else {
            struct walfs_inode ino;
            if (!walfs_stat(id, &ino) || !(ino.flags & WALFS_DIR)) return false;
        }
        parent = id;
    }
    return true;
}

static bool capsule_pack_dir(u32 pack, char *out, u32 out_max)
{
    if (!capsule_pack_valid(pack) || !out || out_max < 24) return false;
    u32 p = 0;
    const char *base = CAPSULE_BASE_DIR "/p";
    while (*base) {
        if (p + 1U >= out_max) return false;
        out[p++] = *base++;
    }
    p += c_u32_dec(&out[p], pack);
    if (p + 1U >= out_max) return false;
    out[p] = 0;
    return true;
}

bool capsule_store_path(u32 pack, u32 card, char *out, u32 out_max)
{
    if (!capsule_pack_dir(pack, out, out_max)) return false;
    u32 p = c_strlen(out);
    const char *role = capsule_card_role(card);
    if (p + 4U >= out_max) return false;
    out[p++] = '/';
    out[p++] = 'c';
    p += c_u32_dec(&out[p], card);
    if (p + 2U + c_strlen(role) >= out_max) return false;
    out[p++] = '.';
    while (*role) out[p++] = *role++;
    out[p] = 0;
    return true;
}

bool capsule_store_init(void)
{
    if (!ensure_dir("/var")) return false;
    return ensure_dir(CAPSULE_BASE_DIR);
}

bool capsule_store_write(u32 pack, u32 card, const void *data, u32 len)
{
    if (!data || len == 0 || len > CAPSULE_STORE_CARD_MAX_BYTES) return false;
    if (!capsule_store_init()) return false;
    char dir[64];
    char path[96];
    if (!capsule_pack_dir(pack, dir, sizeof(dir))) return false;
    if (!ensure_dir(dir)) return false;
    if (!capsule_store_path(pack, card, path, sizeof(path))) return false;
    u64 id = walfs_find(path);
    if (!id) {
        u64 parent = walfs_find(dir);
        char leaf[48];
        u32 n = 0;
        const char *s = path + c_strlen(dir) + 1U;
        while (*s && n + 1U < sizeof(leaf)) leaf[n++] = *s++;
        leaf[n] = 0;
        id = parent ? walfs_create(parent, leaf, WALFS_FILE, 0644) : 0;
        if (!id) return false;
    } else {
        struct walfs_inode ino;
        if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) return false;
    }
    if (!walfs_replace(id, data, len)) return false;

    struct walfs_inode ino;
    if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR) || ino.size != len)
        return false;

    static u8 verify[CAPSULE_STORE_CARD_MAX_BYTES];
    u32 got = walfs_read(id, 0, verify, len);
    if (got != len) return false;
    const u8 *src = (const u8 *)data;
    for (u32 i = 0; i < len; i++)
        if (verify[i] != src[i])
            return false;
    return true;
}

i32 capsule_store_read(u32 pack, u32 card, void *out, u32 out_len)
{
    return capsule_store_read_at(pack, card, 0, out, out_len, NULL);
}

i32 capsule_store_read_at(u32 pack, u32 card, u32 offset, void *out, u32 out_len, u32 *size_out)
{
    if (!out || out_len == 0) return -1;
    char path[96];
    if (!capsule_store_path(pack, card, path, sizeof(path))) return -1;
    u64 id = walfs_find(path);
    if (!id) return -1;
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) return -1;
    if (size_out) *size_out = (ino.size > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (u32)ino.size;
    if (offset > ino.size) return 0;
    u64 avail64 = ino.size - offset;
    u32 avail = (avail64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (u32)avail64;
    u32 n = avail > out_len ? out_len : avail;
    return (i32)walfs_read(id, offset, out, n);
}

bool capsule_store_delete(u32 pack, u32 card)
{
    char path[96];
    if (!capsule_store_path(pack, card, path, sizeof(path))) return false;
    u64 id = walfs_find(path);
    return id && walfs_delete(id);
}

static bool parse_card_name(const u8 *name, u32 *card_out)
{
    if (!name || !card_out || name[0] != 'c') return false;
    u32 v = 0;
    u32 i = 1;
    bool have = false;
    while (name[i] >= '0' && name[i] <= '9') {
        have = true;
        v = v * 10U + (u32)(name[i] - '0');
        i++;
    }
    if (!have || name[i] != '.') return false;
    *card_out = v;
    return true;
}

struct list_ctx {
    u32 *out;
    u32 max;
    u32 n;
};

static struct list_ctx g_list_ctx;

static void list_cb(const struct walfs_dirent *entry)
{
    if (!entry || !g_list_ctx.out || g_list_ctx.n >= g_list_ctx.max) return;
    u32 card = 0;
    if (parse_card_name(entry->name, &card)) {
        for (u32 i = 0; i < g_list_ctx.n; i++)
            if (g_list_ctx.out[i] == card) return;
        g_list_ctx.out[g_list_ctx.n++] = card;
    }
}

u32 capsule_store_list(u32 pack, u32 *out_cards, u32 max_cards)
{
    char dir[64];
    if (!out_cards || !max_cards || !capsule_pack_dir(pack, dir, sizeof(dir))) return 0;
    u64 id = walfs_find(dir);
    if (!id) return 0;
    g_list_ctx.out = out_cards;
    g_list_ctx.max = max_cards;
    g_list_ctx.n = 0;
    walfs_readdir(id, list_cb);
    return g_list_ctx.n;
}

/* capsule_manifest_parse() lives in capsule_manifest_parse.c now (pure
 * logic, host-buildable/fuzzable -- see that file's header comment). */

bool capsule_store_load_manifest(u32 pack, struct capsule_manifest *out,
                                 char *err, u32 err_max)
{
    static char buf[CAPSULE_STORE_CARD_MAX_BYTES];
    i32 n = capsule_store_read(pack, 0, buf, sizeof(buf));
    if (n <= 0) {
        c_err(err, err_max, "manifest card missing");
        return false;
    }
    return capsule_manifest_parse(buf, (u32)n, out, err, err_max);
}
