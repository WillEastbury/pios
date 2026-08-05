/*
 * capsule_manifest_parse.c - pure-logic <path>.cap manifest/card text parser
 *
 * Extracted from capsule_store.c (which also does WALFS I/O, so pulling in
 * that whole file for a host build/fuzz harness would require stubbing out
 * every walfs_* symbol it references). This file touches no MMIO/asm/WALFS
 * and can be compiled standalone on the host, mirroring how dhcp_options.c
 * was carved out of dhcp.c for the same reason (see tests/run_host_tests.py,
 * tests/fuzz_capsule_manifest.c).
 *
 * CORRECTION (post rubber-duck-review): capsule_manifest_parse() is the
 * parser behind capsule_store_load_manifest() only -- the "capsule pack"
 * loader used by kernel.c's console commands and uhttp_bridge.c. It is
 * NOT used by proc.c's capsule_manifest_load(), which is a separate,
 * hand-rolled <path>.cap line/key=value parser that writes directly into
 * struct process fields (quotas, capsule_group, capsule_vfs_root, fs/ipc/
 * pipe prefixes, card/port ranges) rather than the struct capsule_manifest
 * this file parses into. An earlier version of this comment incorrectly
 * claimed the two were the same code path; they are not, so this fuzz
 * harness's coverage does not extend to the mandatory-by-default per-
 * process stage-2 isolation parser in proc.c. That parser remains
 * untested by fuzzing -- unifying the two would require reconciling two
 * different target structs and is a larger, separately-scoped refactor,
 * not a safe surgical fix for this pass. This file is still a real,
 * network/storage-facing text parser worth fuzzing in its own right.
 */

#include "capsule_store.h"
#include "simd.h"

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

static bool c_host_ok(const char *s)
{
    u32 n = 0;
    if (!s || !*s)
        return false;
    while (*s) {
        char ch = *s++;
        bool alpha = (ch >= 'a' && ch <= 'z') ||
                     (ch >= 'A' && ch <= 'Z');
        bool digit = ch >= '0' && ch <= '9';
        if (!alpha && !digit && ch != '-' && ch != '.')
            return false;
        n++;
    }
    return n < CAPSULE_HOST_MAX;
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
            } else if (c_streq(key, "host")) {
                if (!c_host_ok(val) || !c_copy(p->host, sizeof(p->host), val)) {
                    c_err(err, err_max, "bad host");
                    return false;
                }
                p->has_host = true;
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
