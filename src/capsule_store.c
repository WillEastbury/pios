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

static bool c_streq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        a++; b++;
    }
    return *a == 0 && *b == 0;
}

static bool c_starts(const char *s, const char *prefix)
{
    if (!s || !prefix) return false;
    while (*prefix)
        if (*s++ != *prefix++) return false;
    return true;
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

static bool c_parse_u32(const char *s, u32 *out)
{
    if (!s || !*s || !out) return false;
    u32 v = 0;
    while (*s) {
        if (*s < '0' || *s > '9') return false;
        u32 d = (u32)(*s - '0');
        if (v > (0xFFFFFFFFU - d) / 10U) return false;
        v = v * 10U + d;
        s++;
    }
    *out = v;
    return true;
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

static bool c_copy_n(char *dst, u32 max, const char *src, u32 len)
{
    if (!dst || max == 0 || !src || len + 1U > max) return false;
    for (u32 i = 0; i < len; i++) dst[i] = src[i];
    dst[len] = 0;
    return true;
}

static bool c_name_ok(const char *s)
{
    if (!s || !s[0]) return false;
    for (u32 i = 0; s[i]; i++) {
        char c = s[i];
        bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

static void c_trim_copy(char *dst, u32 max, const char *src, u32 len)
{
    if (!dst || max == 0) return;
    while (len && (*src == ' ' || *src == '\t')) {
        src++; len--;
    }
    while (len && (src[len - 1U] == ' ' || src[len - 1U] == '\t')) len--;
    if (len + 1U > max) len = max - 1U;
    for (u32 i = 0; i < len; i++) dst[i] = src[i];
    dst[len] = 0;
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
    if (card >= 10001U && card <= 20000U) return "bytecode";
    if (card >= 20000U) return "ipc";
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
    return walfs_replace(id, data, len);
}

i32 capsule_store_read(u32 pack, u32 card, void *out, u32 out_len)
{
    if (!out || out_len == 0) return -1;
    char path[96];
    if (!capsule_store_path(pack, card, path, sizeof(path))) return -1;
    u64 id = walfs_find(path);
    if (!id) return -1;
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) return -1;
    u32 n = ino.size > out_len ? out_len : (u32)ino.size;
    return (i32)walfs_read(id, 0, out, n);
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
    if (parse_card_name(entry->name, &card))
        g_list_ctx.out[g_list_ctx.n++] = card;
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

static bool parse_range(const char *s, u32 *lo, u32 *hi)
{
    const char *dash = s;
    while (*dash && *dash != '-') dash++;
    if (*dash != '-') return false;
    char a[16], b[16];
    if (!c_copy_n(a, sizeof(a), s, (u32)(dash - s))) return false;
    if (!c_copy(b, sizeof(b), dash + 1)) return false;
    return c_parse_u32(a, lo) && c_parse_u32(b, hi) && *lo <= *hi;
}

bool capsule_manifest_parse(const char *text, u32 len, struct capsule_manifest *out,
                            char *err, u32 err_max)
{
    if (!text || !out || len == 0) {
        c_err(err, err_max, "empty manifest");
        return false;
    }
    simd_zero(out, sizeof(*out));
    out->cards_lo = 1001U;
    out->cards_hi = 20000U;
    u32 pos = 0;
    u32 current = 0; /* 0 header, 1 process, 2 fifo */
    while (pos < len) {
        u32 ls = pos;
        while (pos < len && text[pos] != '\n' && text[pos] != '\r') pos++;
        u32 le = pos;
        while (pos < len && (text[pos] == '\n' || text[pos] == '\r')) pos++;
        if (le <= ls) continue;
        bool indented = (le - ls >= 2U && text[ls] == ' ' && text[ls + 1U] == ' ');
        char line[192];
        c_trim_copy(line, sizeof(line), &text[ls], le - ls);
        if (!line[0]) continue;
        char *eq = line;
        while (*eq && *eq != '=') eq++;
        if (*eq != '=') {
            c_err(err, err_max, "line missing =");
            return false;
        }
        *eq = 0;
        char key[32], val[128];
        c_trim_copy(key, sizeof(key), line, c_strlen(line));
        c_trim_copy(val, sizeof(val), eq + 1, c_strlen(eq + 1));
        if (!indented) {
            if (c_streq(key, "capsule")) {
                out->enabled = c_streq(val, "on") || c_streq(val, "true") || c_streq(val, "1");
                current = 0;
            } else if (c_streq(key, "name")) {
                if (!c_name_ok(val) || !c_copy(out->name, sizeof(out->name), val)) {
                    c_err(err, err_max, "bad capsule name");
                    return false;
                }
                current = 0;
            } else if (c_streq(key, "principal")) {
                if (!c_name_ok(val) || !c_copy(out->principal, sizeof(out->principal), val)) {
                    c_err(err, err_max, "bad principal");
                    return false;
                }
                out->has_principal = true;
                current = 0;
            } else if (c_streq(key, "mem_kib")) {
                if (!c_parse_u32(val, &out->mem_kib)) { c_err(err, err_max, "bad mem_kib"); return false; }
                out->has_mem_kib = true; current = 0;
            } else if (c_streq(key, "cpu_ms")) {
                if (!c_parse_u32(val, &out->cpu_ms)) { c_err(err, err_max, "bad cpu_ms"); return false; }
                out->has_cpu_ms = true; current = 0;
            } else if (c_streq(key, "fs")) {
                if (!c_starts(val, "/") || !c_copy(out->fs, sizeof(out->fs), val)) { c_err(err, err_max, "bad fs"); return false; }
                out->has_fs = true; current = 0;
            } else if (c_streq(key, "cards")) {
                if (!parse_range(val, &out->cards_lo, &out->cards_hi)) { c_err(err, err_max, "bad cards range"); return false; }
                current = 0;
            } else if (c_streq(key, "process")) {
                if (out->process_count >= CAPSULE_PROCESS_MAX || !c_name_ok(val)) { c_err(err, err_max, "bad process"); return false; }
                struct capsule_process *p = &out->processes[out->process_count++];
                simd_zero(p, sizeof(*p));
                (void)c_copy(p->name, sizeof(p->name), val);
                current = 1;
            } else if (c_streq(key, "ipc_fifo")) {
                if (out->fifo_count >= CAPSULE_FIFO_MAX || !c_name_ok(val)) { c_err(err, err_max, "bad fifo"); return false; }
                struct capsule_fifo *f = &out->fifos[out->fifo_count++];
                simd_zero(f, sizeof(*f));
                (void)c_copy(f->name, sizeof(f->name), val);
                current = 2;
            } else {
                c_err(err, err_max, "unknown manifest key");
                return false;
            }
        } else if (current == 1 && out->process_count > 0) {
            struct capsule_process *p = &out->processes[out->process_count - 1U];
            if (c_streq(key, "source")) {
                if (!c_parse_u32(val, &p->source)) { c_err(err, err_max, "bad source"); return false; }
            } else if (c_streq(key, "bytecode")) {
                if (!c_parse_u32(val, &p->bytecode)) { c_err(err, err_max, "bad bytecode"); return false; }
            } else if (c_streq(key, "io")) {
                if (!c_copy(p->io, sizeof(p->io), val)) { c_err(err, err_max, "bad io"); return false; }
                if (c_starts(val, "tcp/")) {
                    u32 port = 0;
                    if (!c_parse_u32(val + 4, &port) || port > 65535U) { c_err(err, err_max, "bad tcp port"); return false; }
                    p->has_tcp = true;
                    p->tcp_port = (u16)port;
                } else if (!c_starts(val, "fifo/")) {
                    c_err(err, err_max, "bad io kind");
                    return false;
                }
            } else if (c_streq(key, "entry")) {
                if (!c_name_ok(val) || !c_copy(p->entry, sizeof(p->entry), val)) { c_err(err, err_max, "bad entry"); return false; }
            } else { c_err(err, err_max, "bad process key"); return false; }
        } else if (current == 2 && out->fifo_count > 0) {
            struct capsule_fifo *f = &out->fifos[out->fifo_count - 1U];
            if (c_streq(key, "from")) {
                if (!c_name_ok(val) || !c_copy(f->from, sizeof(f->from), val)) { c_err(err, err_max, "bad fifo from"); return false; }
            } else if (c_streq(key, "to")) {
                if (!c_name_ok(val) || !c_copy(f->to, sizeof(f->to), val)) { c_err(err, err_max, "bad fifo to"); return false; }
            } else if (c_streq(key, "depth")) {
                if (!c_parse_u32(val, &f->depth)) { c_err(err, err_max, "bad fifo depth"); return false; }
            } else if (c_streq(key, "frame_max")) {
                if (!c_parse_u32(val, &f->frame_max)) { c_err(err, err_max, "bad fifo frame"); return false; }
            } else { c_err(err, err_max, "bad fifo key"); return false; }
        } else {
            c_err(err, err_max, "indented line outside block");
            return false;
        }
    }
    if (!out->enabled || !out->name[0]) { c_err(err, err_max, "missing capsule header"); return false; }
    if (out->process_count == 0) { c_err(err, err_max, "missing process"); return false; }
    return true;
}

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
