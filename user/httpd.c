/*
 * httpd.c - PIOS userland HTTP server (process on a user core).
 *
 * The kernel terminates TCP on port 81 and hands each request to this process
 * via the shared uhttp_bridge at IPC_SHM_BASE. This process SLEEPS (api->park)
 * between requests and is woken by a cross-core software interrupt when a
 * request arrives — no syscall spin, no blocking socket calls. It reads the
 * request, builds the response into the shared buffer, and signals the kernel,
 * which writes it back to the client. Live kernel data (pid, uptime, request
 * count) is read through the kernel_api table at request time.
 *
 * Constraint: the image is mapped read-execute (W^X) — no writable globals; all
 * mutable state is on the stack, in the shared bridge, or (for build_body()'s
 * PicoScript VM state -- see struct httpd_scratch below) a fixed-VA scratch
 * region inside this process's own mapped slot.
 *
 * BUILD REQUIREMENT: this file must be compiled with -fno-gcse (see
 * build_bootstrap.bat). build_body()'s VM branch picks between two `program`
 * pointers (picoweb_default_program vs a loaded card in sc->program_words)
 * before calling pv_vm_run() -- the identical pattern that hit a confirmed
 * GCC 13.3 (aarch64-none-elf) -fgcse miscompile in user/capsvc_host.c (GCC
 * merged the two branches' tails and used the wrong register for the
 * `program` argument at -O2, producing a near-NULL pointer that faults
 * inside pv_verify() with no diagnostic). Applied here preventatively for
 * the same code shape, even though this path was previously untested.
 */
#include "types.h"
#include "proc.h"          /* struct kernel_api */
#include "uhttp_bridge.h"  /* struct uhttp_bridge, UHTTP_BRIDGE_ADDR */
#include "picovm.h"
#include "pico_hooks.h"
#include "walfs.h"
#include "ipc_proc.h"

/* Minimal freestanding mem primitives (also satisfy implicit compiler calls). */
void *memcpy(void *d, const void *s, unsigned long n)
{
    u8 *dd = (u8 *)d; const u8 *ss = (const u8 *)s;
    for (unsigned long i = 0; i < n; i++) dd[i] = ss[i];
    return d;
}
void *memset(void *d, int c, unsigned long n)
{
    u8 *dd = (u8 *)d;
    for (unsigned long i = 0; i < n; i++) dd[i] = (u8)c;
    return d;
}

static u32 u_strlen(const char *s)
{
    u32 n = 0;
    while (s && s[n]) n++;
    return n;
}

static void u_append(char *dst, u32 *off, u32 cap, const char *src)
{
    u32 n = u_strlen(src);
    if (*off + n >= cap) n = (cap > *off) ? (cap - *off - 1) : 0;
    for (u32 i = 0; i < n; i++) dst[*off + i] = src[i];
    *off += n;
    dst[*off] = 0;
}

static void u_append_u32(char *dst, u32 *off, u32 cap, u32 v)
{
    char tmp[12];
    u32 i = 0;
    if (v == 0) tmp[i++] = '0';
    while (v) { tmp[i++] = (char)('0' + (v % 10)); v /= 10; }
    char rev[12];
    for (u32 j = 0; j < i; j++) rev[j] = tmp[i - 1 - j];
    rev[i] = 0;
    u_append(dst, off, cap, rev);
}

#ifdef PIOS_USER_EL0
static inline u32 el0_getpid(void)
{
    register u64 x0 __asm__("x0");
    __asm__ volatile("svc #1" : "=r"(x0) :: "memory");
    return (u32)x0;
}

static inline void el0_wait_event(void)
{
#ifdef PIOS_USER_SPIN_WAIT
    __asm__ volatile("" ::: "memory");
#else
#ifdef PIOS_USER_WFE_WAIT
    __asm__ volatile("wfe" ::: "memory");
#else
    register u64 x0 __asm__("x0");
    __asm__ volatile("svc #4" : "=r"(x0) :: "memory");
#endif
#endif
}
#endif

#define PICOWEB_MEM_SIZE        8192U
#define PICOWEB_MAX_SPANS       64U
#define PICOWEB_CAPSULE_PACK_MIN 1024U
#define PICOWEB_CAPSULE_PACK_MAX 4095U
#define PICOWEB_CAPSULE_CARD_MAX 65535U
#define PICOWEB_CARD_MAX_BYTES   16384U

struct pico_span {
    u32 ptr;
    u32 len;
};

struct picoweb_host {
    pv_ctx vm;                 /* first: pv_host callback receives &vm */
    struct uhttp_bridge *bridge;
    struct kernel_api *api;
    u32 reqs;
    u32 active_pack;
    u8 mem[PICOWEB_MEM_SIZE];
    u32 arena_top;
    struct pico_span spans[PICOWEB_MAX_SPANS];
    int span_count;
    int oom;
};

/* pv_ctx (include/picovm.h) is genuinely large (~65KB card/span/writer/queue
 * tables) -- struct picoweb_host measures ~80KB via -Wframe-larger-than, and
 * build_body() also needs a UHTTP_PICO_MAX/4 (4KB) program-word buffer on top
 * of that. EL0 processes launched via proc_exec_from_mem_el0() get only a
 * tiny fixed stack (entry_sp = linked_base + 0x20000, 128KB, shared downward
 * with the loaded image -- roughly 51KB of real headroom here), so neither of
 * these may ever be stack-allocated: doing so blows straight through the
 * stack and corrupts the loaded code with no clean trap (an EC=0 "unknown
 * reason" illegal-instruction fault on the very first real request; measured
 * and reproduced while integrating user/capsvc_host.c's PicoScript execution,
 * which hit the identical bug and fixed it the same way). Instead both live
 * at a fixed VA inside this process's own mapped 2MiB slot, past the loaded
 * image and its stack -- proc_exec_from_mem_el0()/
 * mmu_user_table_build_split_el0_at() already map and zero the ENTIRE slot
 * (not just the loaded bytes), so this is valid, private, RW, zero-
 * initialized memory reachable with plain loads/stores -- not a "writable
 * global" in the W^X sense (no static/global storage-class object exists;
 * it is simply this process's own unused mapped RAM, reinterpreted), and
 * single-threaded (one request at a time), so one shared scratch buffer at a
 * well-known VA is safe. HTTPD_IMAGE_LINK_BASE must match user/httpd_el0.ld's
 * fixed link VA (shared by both UHTTP_BRIDGE_INDEX=0 and =1 builds, which
 * are separate processes/address spaces, never concurrent with each other). */
#define HTTPD_IMAGE_LINK_BASE 0x2001000000ULL
#define HTTPD_HEAP_OFFSET     0x100000ULL   /* 1MiB in: comfortably past this ~77KB image + stack, comfortably inside the 2MiB slot */
#define HTTPD_HEAP_VA         (HTTPD_IMAGE_LINK_BASE + HTTPD_HEAP_OFFSET)

struct httpd_scratch {
    struct picoweb_host host;
    u32 program_words[UHTTP_PICO_MAX / 4U];
};

static const u32 picoweb_default_program[] = {
    0x000070E1u, /* Context.GetPath(R0) */
    0x41100001u, /* Math.Add(R1, R1, 1) */
    0x02017044u, /* Span.Get(R0, R1, R2) */
    0x4330006Cu, /* Math.Add(R3, R3, 'l') */
    0xA230000Fu, /* Flow.Branch(EQ, R2, R3, :large) */
    0x4330FFF5u, /* Math.Add(R3, R3, -11) => 'a' */
    0xA2300007u, /* Flow.Branch(EQ, R2, R3, :api) */
    0x4440019Au, /* Math.Add(R4, R4, 410) */
    0x45500001u, /* Math.Add(R5, R5, 1) */
    0x06457066u, /* Storage.ReadCard(R4, R5, R6) */
    0x00607071u, /* Io.Write(R6) */
    0x000080C8u, /* Net.Status(200) */
    0xC0000000u, /* Flow.Return() */
    0x4440019Au, /* Math.Add(R4, R4, 410) */
    0x45500002u, /* Math.Add(R5, R5, 2) */
    0x06457066u, /* Storage.ReadCard(R4, R5, R6) */
    0x00607071u, /* Io.Write(R6) */
    0x000080C8u, /* Net.Status(200) */
    0xC0000000u, /* Flow.Return() */
    0x4440019Au, /* :large Math.Add(R4, R4, 410) */
    0x45500003u, /* Math.Add(R5, R5, 3) */
    0x06457066u, /* Storage.ReadCard(R4, R5, R6) */
    0x00607071u, /* Io.Write(R6) */
    0x000080C8u, /* Net.Status(200) */
    0xC0000000u, /* Flow.Return() */
};

static int pico_alloc_span(struct picoweb_host *h, const u8 *src, u32 n)
{
    if (h->span_count >= (int)PICOWEB_MAX_SPANS) { h->oom = 1; return 0; }
    if (n > PICOWEB_MEM_SIZE - h->arena_top) { h->oom = 1; n = PICOWEB_MEM_SIZE - h->arena_top; }
    u32 ptr = h->arena_top;
    for (u32 i = 0; i < n; i++) h->mem[ptr + i] = src[i];
    h->arena_top += n;
    int handle = h->span_count++;
    h->spans[handle].ptr = ptr;
    h->spans[handle].len = n;
    return handle;
}

static int pico_alloc_span_direct(struct picoweb_host *h, u32 ptr, u32 n)
{
    if (h->span_count >= (int)PICOWEB_MAX_SPANS) { h->oom = 1; return 0; }
    int handle = h->span_count++;
    h->spans[handle].ptr = ptr;
    h->spans[handle].len = n;
    return handle;
}

static void pico_write_span(struct picoweb_host *h, i32 handle)
{
    if (handle <= 0 || handle >= h->span_count) return;
    struct pico_span s = h->spans[handle];
    for (u32 i = 0; i < s.len && h->vm.out_len < PV_MAX_OUT; i++)
        h->vm.out[h->vm.out_len++] = h->mem[s.ptr + i];
}

static bool pico_text_ok(const u8 *p, u32 n, u8 first)
{
    if (!p || n == 0 || p[0] != first)
        return false;
    for (u32 i = 0; i < n; i++) {
        u8 c = p[i];
        if (c == '\r' || c == '\n' || c == '\t')
            continue;
        if (c < 0x20U || c > 0x7EU)
            return false;
    }
    return true;
}

    static bool pico_pack_ok(u32 pack)
    {
        return pack >= PICOWEB_CAPSULE_PACK_MIN && pack <= PICOWEB_CAPSULE_PACK_MAX;
    }

    static const char *pico_card_role(u32 card)
    {
        if (card == 0) return "manifest";
        if (card >= 1U && card <= 1000U) return "exec";
        if (card >= 1001U && card <= 10000U) return "source";
        if (card >= 20000U) return "ipc";
        if (card >= 10001U) return "bytecode";
        return "card";
    }

    static bool pico_append(char *dst, u32 *off, u32 cap, const char *src)
    {
        while (*src) {
            if (*off + 1U >= cap) return false;
            dst[(*off)++] = *src++;
        }
        dst[*off] = 0;
        return true;
    }

    static bool pico_append_u32(char *dst, u32 *off, u32 cap, u32 v)
    {
        char tmp[12];
        u32 n = 0;
        if (v == 0) tmp[n++] = '0';
        while (v && n < sizeof(tmp)) {
            tmp[n++] = (char)('0' + (v % 10U));
            v /= 10U;
        }
        while (n) {
            if (*off + 1U >= cap) return false;
            dst[(*off)++] = tmp[--n];
        }
        dst[*off] = 0;
        return true;
    }

    static bool pico_capsule_pack_dir(u32 pack, char *out, u32 out_max)
    {
        u32 off = 0;
        return pico_pack_ok(pack) &&
               pico_append(out, &off, out_max, "/var/capsules/p") &&
               pico_append_u32(out, &off, out_max, pack);
    }

    static bool pico_capsule_card_path(u32 pack, u32 card, char *out, u32 out_max)
    {
        u32 off = 0;
        if (!pico_pack_ok(pack) || card > PICOWEB_CAPSULE_CARD_MAX) return false;
        return pico_append(out, &off, out_max, "/var/capsules/p") &&
               pico_append_u32(out, &off, out_max, pack) &&
               pico_append(out, &off, out_max, "/c") &&
               pico_append_u32(out, &off, out_max, card) &&
               pico_append(out, &off, out_max, ".") &&
               pico_append(out, &off, out_max, pico_card_role(card));
    }

    static int pico_card_address_span(struct picoweb_host *h, u32 pack, u32 card)
    {
        char addr[32];
        u32 off = 0;
        if (!pico_pack_ok(pack) || card > PICOWEB_CAPSULE_CARD_MAX) return 0;
        if (!pico_append_u32(addr, &off, sizeof(addr), pack) ||
            !pico_append(addr, &off, sizeof(addr), "/") ||
            !pico_append_u32(addr, &off, sizeof(addr), card))
            return 0;
        return pico_alloc_span(h, (const u8 *)addr, off);
    }

    static void pico_ensure_capsule_dirs(struct picoweb_host *h, u32 pack)
    {
        if (!h->api) return;
        char dir[64];
        if (!pico_capsule_pack_dir(pack, dir, sizeof(dir))) return;
        (void)h->api->mkdir("/var");
        (void)h->api->mkdir("/var/capsules");
        (void)h->api->mkdir(dir);
    }

    static int pico_card_read_span(struct picoweb_host *h, u32 card)
    {
        if (!h->api) { h->vm.host_status = 20; return 0; }
        char path[96];
        if (!pico_capsule_card_path(h->active_pack, card, path, sizeof(path))) { h->vm.host_status = 21; return 0; }
        struct walfs_inode ino;
        if (h->api->stat(path, &ino) != 0 || (ino.flags & WALFS_DIR)) { h->vm.host_status = 22; return 0; }
        u32 n = (ino.size > PICOWEB_CARD_MAX_BYTES) ? PICOWEB_CARD_MAX_BYTES : (u32)ino.size;
        if (n > PICOWEB_MEM_SIZE - h->arena_top) { h->oom = 1; h->vm.host_status = 23; return 0; }
        i32 fd = h->api->open(path, 0);
        if (fd < 0) { h->vm.host_status = 24; return 0; }
        u32 ptr = h->arena_top;
        i32 got = h->api->pread(fd, &h->mem[ptr], n, 0);
        (void)h->api->close(fd);
        if (got < 0) { h->vm.host_status = 25; return 0; }
        h->arena_top += (u32)got;
        h->vm.host_status = 0;
        return pico_alloc_span_direct(h, ptr, (u32)got);
    }

    static bool pico_card_verify(struct picoweb_host *h, const char *path, const u8 *data, u32 len)
    {
        u8 tmp[256];
        i32 fd = h->api->open(path, 0);
        if (fd < 0) return false;
        u32 off = 0;
        while (off < len) {
            u32 chunk = len - off;
            if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
            i32 got = h->api->pread(fd, tmp, chunk, off);
            if (got != (i32)chunk) { (void)h->api->close(fd); return false; }
            for (u32 i = 0; i < chunk; i++) {
                if (tmp[i] != data[off + i]) { (void)h->api->close(fd); return false; }
            }
            off += chunk;
        }
        struct walfs_inode ino;
        bool ok = h->api->stat(path, &ino) == 0 && ino.size == len;
        (void)h->api->close(fd);
        return ok;
    }

    static int pico_card_write_span(struct picoweb_host *h, u32 card, i32 handle)
    {
        if (!h->api) { h->vm.host_status = 10; return 0; }
        if (handle <= 0 || handle >= h->span_count) { h->vm.host_status = 11; return 0; }
        struct pico_span s = h->spans[handle];
        if (s.len == 0 || s.len > PICOWEB_CARD_MAX_BYTES) { h->vm.host_status = 12; return 0; }
        char path[96];
        if (!pico_capsule_card_path(h->active_pack, card, path, sizeof(path))) { h->vm.host_status = 13; return 0; }
        pico_ensure_capsule_dirs(h, h->active_pack);
        (void)h->api->unlink(path);
        i32 fd = h->api->creat(path, 0, 0644);
        if (fd < 0) { h->vm.host_status = 14; return 0; }
        i32 wrote = h->api->pwrite(fd, &h->mem[s.ptr], s.len, 0);
        (void)h->api->close(fd);
        if (wrote != (i32)s.len) { h->vm.host_status = 15; return 0; }
        if (!pico_card_verify(h, path, &h->mem[s.ptr], s.len)) { h->vm.host_status = 16; return 0; }
        h->vm.host_status = 0;
        return 1;
    }

    static bool pico_span_name(struct picoweb_host *h, i32 handle, char *out, u32 out_max)
    {
        if (handle <= 0 || handle >= h->span_count || !out || out_max < 2) return false;
        struct pico_span s = h->spans[handle];
        if (s.len == 0 || s.len + 1U > out_max) return false;
        for (u32 i = 0; i < s.len; i++) {
            char c = (char)h->mem[s.ptr + i];
            bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                      (c >= '0' && c <= '9') || c == '_' || c == '-';
            if (!ok) return false;
            out[i] = c;
        }
        out[s.len] = 0;
        return true;
    }

    static int pico_fifo_open(struct picoweb_host *h, i32 name_span)
    {
        if (!h->api) { h->vm.host_status = 30; return -1; }
        char name[PROC_IPC_NAME_MAX + 1];
        if (!pico_span_name(h, name_span, name, sizeof(name))) { h->vm.host_status = 31; return -1; }
        i32 hnd = h->api->ipc_fifo_open(name, PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV);
        if (hnd >= 0) { h->vm.host_status = 0; return hnd; }
        hnd = h->api->ipc_fifo_create(name, PROC_IPC_PEER_ANY,
                                      PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV,
                                      PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV,
                                      PROC_IPC_FIFO_DEPTH_MAX, PROC_IPC_FIFO_MSG_MAX);
        if (hnd < 0) { h->vm.host_status = 32; return hnd; }
        hnd = h->api->ipc_fifo_open(name, PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV);
        h->vm.host_status = (hnd >= 0) ? 0 : 33;
        return hnd;
    }

    static int pico_fifo_recv_span(struct picoweb_host *h, i32 fifo)
    {
        if (!h->api) { h->vm.host_status = 30; return 0; }
        if (!h->api || h->arena_top >= PICOWEB_MEM_SIZE) return 0;
        u32 ptr = h->arena_top;
        u32 cap = PICOWEB_MEM_SIZE - ptr;
        if (cap > PROC_IPC_FIFO_MSG_MAX) cap = PROC_IPC_FIFO_MSG_MAX;
        i32 got = h->api->ipc_fifo_recv(fifo, &h->mem[ptr], cap);
        if (got <= 0) return 0;
        h->arena_top += (u32)got;
        return pico_alloc_span_direct(h, ptr, (u32)got);
    }

static void req_bounds(struct uhttp_bridge *b, u32 *method_s, u32 *method_n,
                       u32 *path_s, u32 *path_n, u32 *query_s, u32 *query_n,
                       u32 *body_s, u32 *body_n)
{
    u32 n = b->req_len;
    if (n > UHTTP_REQ_MAX) n = UHTTP_REQ_MAX;
    u32 i = 0;
    while (i < n && b->req[i] != ' ') i++;
    *method_s = 0; *method_n = i;
    u32 ps = (i < n) ? i + 1U : n;
    u32 pe = ps;
    while (pe < n && b->req[pe] != ' ') pe++;
    u32 qs = pe, qe = pe;
    for (u32 j = ps; j < pe; j++) {
        if (b->req[j] == '?') { qs = j + 1U; qe = pe; pe = j; break; }
    }
    *path_s = ps; *path_n = (pe >= ps) ? pe - ps : 0;
    *query_s = qs; *query_n = (qe >= qs) ? qe - qs : 0;
    *body_s = n; *body_n = 0;
    for (u32 j = 3; j < n; j++) {
        if (b->req[j - 3] == '\r' && b->req[j - 2] == '\n' &&
            b->req[j - 1] == '\r' && b->req[j] == '\n') {
            *body_s = j + 1U;
            *body_n = n - *body_s;
            break;
        }
    }
}

static int pico_req_span(struct picoweb_host *h, int which)
{
    u32 ms, mn, ps, pn, qs, qn, bs, bn;
    req_bounds(h->bridge, &ms, &mn, &ps, &pn, &qs, &qn, &bs, &bn);
    if (which == PV_HOOK_CONTEXT_GETVERB) return pico_alloc_span(h, h->bridge->req + ms, mn);
    if (which == PV_HOOK_CONTEXT_GETPATH) return pico_alloc_span(h, h->bridge->req + ps, pn);
    if (which == PV_HOOK_CONTEXT_GETQUERYSTRING) return pico_alloc_span(h, h->bridge->req + qs, qn);
    if (which == PV_HOOK_CONTEXT_GETBODY) return pico_alloc_span(h, h->bridge->req + bs, bn);
    return pico_alloc_span(h, (const u8 *)"", 0);
}

#ifdef PIOS_HTTPD_NATIVE
static bool req_path_is(struct uhttp_bridge *b, const char *want);
#endif
static u32 build_large_dynamic(char *buf, u32 cap, u32 reqs, const char *engine);

static int pico_read_card_span(struct picoweb_host *h, u32 card, u32 record)
{
    uhttp_inval(&h->bridge->pico_prog_len, UHTTP_LINE);
    if (card == UHTTP_PICOWEB_CARD && record == UHTTP_PICO_DYNAMIC_RECORD) {
        u32 ptr = h->arena_top;
        char *dst = (char *)&h->mem[ptr];
        u32 n = build_large_dynamic(dst, PICOWEB_MEM_SIZE - ptr, h->reqs, "picovm");
        return pico_alloc_span(h, (const u8 *)dst, n);
    }
    if (card == UHTTP_PICOWEB_CARD && record == UHTTP_PICO_STATIC_RECORD && h->bridge->pico_static_len > 0) {
        u32 n = h->bridge->pico_static_len;
        if (n > UHTTP_PICO_MAX) n = UHTTP_PICO_MAX;
        uhttp_inval(h->bridge->pico_static, n);
        if (pico_text_ok(h->bridge->pico_static, n, '<'))
            return pico_alloc_span(h, h->bridge->pico_static, n);
    }
    if (card == UHTTP_PICOWEB_CARD && record == UHTTP_PICO_API_RECORD && h->bridge->pico_api_len > 0) {
        u32 n = h->bridge->pico_api_len;
        if (n > UHTTP_PICO_MAX) n = UHTTP_PICO_MAX;
        uhttp_inval(h->bridge->pico_api, n);
        if (pico_text_ok(h->bridge->pico_api, n, '{'))
            return pico_alloc_span(h, h->bridge->pico_api, n);
    }
    if (card == UHTTP_PICOWEB_CARD && record == UHTTP_PICO_API_RECORD) {
        const char *s = "{\"ok\":true,\"handler\":\"picoscript\",\"source\":\"fallback-card\",\"card\":410,\"record\":2}\n";
        return pico_alloc_span(h, (const u8 *)s, u_strlen(s));
    }
    const char *s =
        "<!doctype html><html><head><meta charset='utf-8'><title>PIOS PicoScript web</title>"
        "<style>body{font-family:'Segoe UI',Aptos,Calibri,sans-serif;background:#3d3b3a;color:#dedede;"
        "margin:0}.wrap{max-width:760px;margin:0 auto;padding:32px}.card{background:#292929;"
        "border:1px solid #474747;border-radius:16px;padding:24px;margin:16px 0}h1{color:#fd8ea1}"
        ".muted{color:#919191}</style></head><body><div class='wrap'><div class='card'>"
        "<h1>PIOS PicoScript webserver</h1><p class='muted'>Port 81 is served by a user process "
        "running upstream picovm bytecode. Core0 preloads program/static/API records from card 410 "
        "before waking this process; this fallback appears when record 410:1 is empty.</p>"
        "<p>Try <a href='/api'>/api</a> for the JSON card route.</p></div></div></body></html>";
    return pico_alloc_span(h, (const u8 *)s, u_strlen(s));
}

static void picoweb_hook(pv_ctx *ctx, int hook, int rd, int rs1, int rs2, int imm16)
{
    struct picoweb_host *h = (struct picoweb_host *)ctx;
    (void)imm16;
    switch (hook) {
    case PV_HOOK_CONTEXT_GETVERB:
    case PV_HOOK_CONTEXT_GETPATH:
    case PV_HOOK_CONTEXT_GETQUERYSTRING:
    case PV_HOOK_CONTEXT_GETBODY:
    case PV_HOOK_CONTEXT_GETHOST:
    case PV_HOOK_CONTEXT_GETHEADERS:
    case PV_HOOK_CONTEXT_GETREMOTEADDR:
        ctx->regs[rd] = pico_req_span(h, hook);
        return;
    case PV_HOOK_CONTEXT_GETPORT:
#if defined(UHTTP_BRIDGE_INDEX) && UHTTP_BRIDGE_INDEX == 1
        ctx->regs[rd] = UHTTP_NATIVE_PORT;
#else
        ctx->regs[rd] = UHTTP_PORT;
#endif
        return;
    case PV_HOOK_CONTEXT_GETUSER:
        ctx->regs[rd] = 0;
        return;
    case PV_HOOK_CONTEXT_GETPERMISSIONS:
        ctx->regs[rd] = 0x1F;
        return;
    case PV_HOOK_CONTEXT_GETREQUESTID:
        ctx->regs[rd] = (i32)h->reqs;
        return;
    case PV_HOOK_STORAGE_READCARD:
        ctx->regs[rd] = pico_read_card_span(h, (u32)ctx->regs[rs1], (u32)ctx->regs[rs2]);
        return;
    case PV_HOOK_IO_WRITE:
        pico_write_span(h, ctx->regs[rs1]);
        return;
    case PV_HOOK_IO_WRITEBYTE:
        if (ctx->out_len < PV_MAX_OUT) ctx->out[ctx->out_len++] = (u8)(ctx->regs[rs1] & 0xFF);
        return;
    case PV_HOOK_SPAN_MAKE: {
        u32 ptr = (u32)ctx->regs[rs1];
        i32 len = ctx->regs[rs2];
        if (len < 0) len = 0;
        if (PICOWEB_MEM_SIZE) ptr %= PICOWEB_MEM_SIZE;
        if ((u32)len > PICOWEB_MEM_SIZE - ptr)
            len = (i32)(PICOWEB_MEM_SIZE - ptr);
        ctx->regs[rd] = pico_alloc_span_direct(h, ptr, (u32)len);
        return;
    }
    case PV_HOOK_SPAN_LEN: {
        i32 hd = ctx->regs[rs1];
        ctx->regs[rd] = (hd > 0 && hd < h->span_count) ? (i32)h->spans[hd].len : 0;
        return;
    }
    case PV_HOOK_SPAN_GET: {
        i32 hd = ctx->regs[rs1], idx = ctx->regs[rs2];
        ctx->regs[rd] = (hd > 0 && hd < h->span_count && idx >= 0 && (u32)idx < h->spans[hd].len)
                            ? h->mem[h->spans[hd].ptr + (u32)idx] : 0;
        return;
    }
    case PV_HOOK_SPAN_SLICE: {
        i32 hd = ctx->regs[rs1], off = ctx->regs[rs2];
        struct pico_span s = (hd > 0 && hd < h->span_count) ? h->spans[hd] : (struct pico_span){0, 0};
        if (off < 0) off = 0;
        if ((u32)off > s.len) off = (i32)s.len;
        if (h->span_count >= (int)PICOWEB_MAX_SPANS) { h->oom = 1; ctx->regs[rd] = 0; return; }
        int handle = h->span_count++;
        h->spans[handle].ptr = s.ptr + (u32)off;
        h->spans[handle].len = s.len - (u32)off;
        ctx->regs[rd] = handle;
        return;
    }
    case PV_HOOK_SPAN_MATERIALIZE: {
        i32 hd = ctx->regs[rs1];
        if (hd <= 0 || hd >= h->span_count) { ctx->regs[rd] = 0; return; }
        struct pico_span s = h->spans[hd];
        ctx->regs[rd] = pico_alloc_span(h, &h->mem[s.ptr], s.len);
        return;
    }
    case PV_HOOK_PACK_USE: {
        u32 pack = (u32)ctx->regs[rs1];
        if (pico_pack_ok(pack)) {
            h->active_pack = pack;
            ctx->regs[rd] = 1;
        } else {
            ctx->regs[rd] = 0;
        }
        return;
    }
    case PV_HOOK_CARD_READ:
        ctx->regs[rd] = pico_card_read_span(h, (u32)ctx->regs[rs1]);
        return;
    case PV_HOOK_CARD_WRITE:
        ctx->regs[rd] = pico_card_write_span(h, (u32)ctx->regs[rs1], ctx->regs[rs2]);
        return;
    case PV_HOOK_CARD_ADDRESS:
        ctx->regs[rd] = pico_card_address_span(h, (u32)ctx->regs[rs1], (u32)ctx->regs[rs2]);
        return;
    case PV_HOOK_FIFO_OPEN:
        ctx->regs[rd] = pico_fifo_open(h, ctx->regs[rs1]);
        return;
    case PV_HOOK_FIFO_SEND: {
        i32 hd = ctx->regs[rs2];
        if (h->api && hd > 0 && hd < h->span_count) {
            struct pico_span s = h->spans[hd];
            ctx->regs[rd] = (h->api->ipc_fifo_send(ctx->regs[rs1], &h->mem[s.ptr], s.len) == PROC_IPC_OK) ? 1 : 0;
        } else {
            h->vm.host_status = 30;
            ctx->regs[rd] = 0;
        }
        return;
    }
    case PV_HOOK_FIFO_RECV:
        ctx->regs[rd] = pico_fifo_recv_span(h, ctx->regs[rs1]);
        return;
    case PV_HOOK_FIFO_POLL:
        ctx->regs[rd] = h->api ? h->api->ipc_fifo_poll(ctx->regs[rs1]) : 0;
        if (!h->api) h->vm.host_status = 30;
        return;
    default:
        pv_default_host(ctx, hook, rd, rs1, rs2, imm16);
        return;
    }
}

static void picoweb_init(struct picoweb_host *h, struct uhttp_bridge *b, struct kernel_api *api, u32 reqs)
{
    pv_init(&h->vm);
    h->vm.host = picoweb_hook;
    h->vm.mem = h->mem;
    h->vm.mem_size = PICOWEB_MEM_SIZE;
    h->bridge = b;
    h->api = api;
    h->reqs = reqs;
    h->active_pack = PICOWEB_CAPSULE_PACK_MIN;
    h->arena_top = 0;
    for (u32 i = 0; i < PICOWEB_MEM_SIZE; i++) h->mem[i] = 0;
    h->span_count = 1;
    h->spans[0].ptr = 0;
    h->spans[0].len = 0;
    h->oom = 0;
}

static bool req_is_api(struct uhttp_bridge *b)
{
    u32 ms, mn, ps, pn, qs, qn, bs, bn;
    req_bounds(b, &ms, &mn, &ps, &pn, &qs, &qn, &bs, &bn);
    (void)ms; (void)mn; (void)qs; (void)qn; (void)bs; (void)bn;
    return pn >= 4 && b->req[ps + 0] == '/' && b->req[ps + 1] == 'a' &&
           b->req[ps + 2] == 'p' && b->req[ps + 3] == 'i';
}

#ifdef PIOS_HTTPD_NATIVE
static bool req_path_is(struct uhttp_bridge *b, const char *want)
{
    u32 ms, mn, ps, pn, qs, qn, bs, bn;
    req_bounds(b, &ms, &mn, &ps, &pn, &qs, &qn, &bs, &bn);
    (void)ms; (void)mn; (void)qs; (void)qn; (void)bs; (void)bn;
    u32 wn = u_strlen(want);
    if (pn != wn) return false;
    for (u32 i = 0; i < wn; i++)
        if ((char)b->req[ps + i] != want[i])
            return false;
    return true;
}
#endif

static u32 build_large_dynamic(char *buf, u32 cap, u32 reqs, const char *engine)
{
    u32 off = 0;
    u_append(buf, &off, cap, "<!doctype html><html><head><meta charset='utf-8'><title>PIOS dynamic benchmark</title>"
             "<style>body{font-family:'Segoe UI',Aptos,Calibri,sans-serif;background:#3d3b3a;color:#dedede;margin:0}"
             ".wrap{max-width:900px;margin:0 auto;padding:24px}.card{background:#292929;border:1px solid #474747;"
             "border-radius:16px;padding:18px;margin:12px 0}h1{color:#fd8ea1}.muted{color:#919191}"
             "td{border-bottom:1px solid #474747;padding:6px 10px}</style></head><body><div class='wrap'><div class='card'>"
             "<h1>PIOS dynamic benchmark</h1><p class='muted'>engine=");
    u_append(buf, &off, cap, engine);
    u_append(buf, &off, cap, " request=");
    u_append_u32(buf, &off, cap, reqs);
    u_append(buf, &off, cap, "</p></div><div class='card'><table>");
    for (u32 i = 0; i < 24; i++) {
        u_append(buf, &off, cap, "<tr><td>row ");
        u_append_u32(buf, &off, cap, i);
        u_append(buf, &off, cap, "</td><td>value ");
        u_append_u32(buf, &off, cap, (reqs * 37U) + i * 13U);
        u_append(buf, &off, cap, "</td><td>EL0 PicoScript/native parity benchmark payload</td></tr>");
    }
    u_append(buf, &off, cap, "</table></div></div></body></html>");
    return off;
}

static u32 load_program(struct uhttp_bridge *b, u32 *words, u32 max_words)
{
    uhttp_inval(&b->pico_prog_len, UHTTP_LINE);
    u32 n = b->pico_prog_len;
    if (n == 0 || n > UHTTP_PICO_MAX || (n & 3U) != 0)
        return 0;
    uhttp_inval(b->pico_prog, n);
    u32 nw = (u32)n / 4U;
    if (nw > max_words) nw = max_words;
    for (u32 i = 0; i < nw; i++)
        words[i] = (u32)b->pico_prog[i * 4 + 0] |
                   ((u32)b->pico_prog[i * 4 + 1] << 8) |
                   ((u32)b->pico_prog[i * 4 + 2] << 16) |
                   ((u32)b->pico_prog[i * 4 + 3] << 24);
    return nw;
}

static u32 build_body(struct uhttp_bridge *b, struct kernel_api *api, char *buf, u32 cap, u32 reqs, u32 *status_out)
{
#ifdef PIOS_HTTPD_NATIVE
    if (status_out) *status_out = 200U;
    if (req_is_api(b)) {
        const char *s = "{\"ok\":true,\"handler\":\"native-c\",\"source\":\"dynamic\",\"record\":2}\n";
        u32 n = u_strlen(s);
        if (n >= cap) n = cap - 1U;
        for (u32 i = 0; i < n; i++) buf[i] = s[i];
        buf[n] = 0;
        return n;
    }
    if (req_path_is(b, "/large"))
        return build_large_dynamic(buf, cap, reqs, "native-c");
    uhttp_inval(&b->pico_static_len, UHTTP_LINE);
    if (b->pico_static_len > 0 && b->pico_static_len < cap) {
        uhttp_inval(b->pico_static, b->pico_static_len);
        if (pico_text_ok(b->pico_static, b->pico_static_len, '<')) {
            for (u32 i = 0; i < b->pico_static_len; i++) buf[i] = (char)b->pico_static[i];
            buf[b->pico_static_len] = 0;
            return b->pico_static_len;
        }
    }
    const char *fallback = "<html><body><h1>native tiny fallback</h1></body></html>";
    u32 n = u_strlen(fallback);
    if (n >= cap) n = cap - 1U;
    for (u32 i = 0; i < n; i++) buf[i] = fallback[i];
    buf[n] = 0;
    return n;
#else
    /* Fixed scratch region, not a stack local -- see struct httpd_scratch's
     * comment above. */
    struct httpd_scratch *sc = (struct httpd_scratch *)(usize)HTTPD_HEAP_VA;
    struct picoweb_host *h = &sc->host;
    const u32 *program = picoweb_default_program;
    u32 nwords = (u32)(sizeof(picoweb_default_program) / sizeof(picoweb_default_program[0]));
    u32 loaded = load_program(b, sc->program_words, UHTTP_PICO_MAX / 4U);
    if (loaded > 0) {
        program = sc->program_words;
        nwords = loaded;
    }

    picoweb_init(h, b, api, reqs);
    (void)pv_vm_run(&h->vm, program, (int)nwords);
    if (h->vm.fault != PV_FAULT_NONE) {
        const char *fault = "PicoScript fault\n";
        u32 fn = u_strlen(fault);
        if (fn >= cap) fn = cap - 1U;
        for (u32 i = 0; i < fn; i++) buf[i] = fault[i];
        buf[fn] = 0;
        if (status_out) *status_out = 500U;
        return fn;
    }
    u32 st = (h->vm.http_status >= 100 && h->vm.http_status <= 599) ? (u32)h->vm.http_status : 200U;
    if (status_out) *status_out = st;
    u32 n = (u32)h->vm.out_len;
    if (n >= cap) n = cap - 1U;
    for (u32 i = 0; i < n; i++) buf[i] = (char)h->vm.out[i];
    buf[n] = 0;
    return n;
#endif
}

void user_main(struct kernel_api *api)
{
    struct uhttp_bridge *b = (struct uhttp_bridge *)(usize)UHTTP_BRIDGE_ADDR;

    /* Prove process entry with a constant store that needs NO kernel API call,
     * before touching the api table or the UART. If magic appears but pid stays
     * 0, we entered but api->getpid() stalled; if magic stays 0, we never ran. */
    b->magic = UHTTP_BRIDGE_MAGIC;

    b->resp_seq = 0;
#ifdef PIOS_USER_EL0
    (void)api;
    b->httpd_pid = el0_getpid();
#else
    /* Now exercise the kernel API: publish our pid so the kernel can wake us. */
    b->httpd_pid = api->getpid();
#endif
    /* Push the whole Zone B control line to PoC so core 0 sees magic/pid even
     * though this core is about to park and never evict the line on its own. */
    uhttp_clean(&b->magic, UHTTP_LINE);

    /* Write-once description, own dedicated line (Zone C) -- a plain string
     * compiled into THIS binary (differs by UHTTP_BRIDGE_INDEX), read by the
     * kernel-side dashboard/"services" command. Never hardcoded by the
     * launcher: it genuinely lives in this .c file. */
    {
#if defined(UHTTP_BRIDGE_INDEX) && UHTTP_BRIDGE_INDEX == 1
        const char *desc = "PicoScript VM HTTP worker (bridge 1, :83)";
#else
        const char *desc = "PicoScript VM HTTP worker (bridge 0, :82)";
#endif
        u32 n = u_strlen(desc);
        if (n >= UHTTP_LINE) n = UHTTP_LINE - 1U;
        for (u32 i = 0; i < n; i++) b->description[i] = desc[i];
        b->description[n] = 0;
        uhttp_clean(b->description, UHTTP_LINE);
    }

#ifndef PIOS_USER_EL0
    api->print("[httpd] attached, waiting on :81\n");
#endif

    uhttp_inval(&b->req_seq, UHTTP_LINE);
    u32 last = b->req_seq;
    for (;;) {
        b->dbg_loops++;
        b->dbg_phase = 10U;                     /* loop top: alive, about to poll */
        uhttp_clean(&b->magic, UHTTP_LINE);     /* publish dbg_loops/dbg_phase now */
        uhttp_inval(&b->req_seq, UHTTP_LINE);   /* refresh Zone A: req_seq/reqs_total */
        u32 cur = b->req_seq;
        if (cur == last) {
            b->dbg_phase = 12U;                 /* parking (no new request) */
            uhttp_clean(&b->magic, UHTTP_LINE); /* publish dbg_loops/resp_seq, then sleep */
#ifdef PIOS_USER_EL0
            el0_wait_event();     /* kernel wake path sends SEV when request arrives */
#else
            api->park();          /* sleep until the kernel wakes us */
#endif
            b->dbg_phase = 13U;                 /* resumed from park */
            uhttp_clean(&b->magic, UHTTP_LINE);
            continue;
        }

        b->dbg_phase = 20U;                     /* request seen: building body */
        uhttp_clean(&b->magic, UHTTP_LINE);
        u32 reqs = b->reqs_total + 1U;
        char body[3072];
        u32 status = 200U;
        bool is_api = req_is_api(b);
        u32 blen = build_body(b, api, body, sizeof(body), reqs, &status);

        b->dbg_phase = 21U;                     /* body built: writing response */
        uhttp_clean(&b->magic, UHTTP_LINE);
        char *r = (char *)b->resp;
        u32 off = 0;
        u_append(r, &off, UHTTP_RESP_MAX, "HTTP/1.0 ");
        u_append_u32(r, &off, UHTTP_RESP_MAX, status);
        u_append(r, &off, UHTTP_RESP_MAX, " OK\r\nContent-Type: ");
        u_append(r, &off, UHTTP_RESP_MAX, is_api ? "application/json" : "text/html");
        u_append(r, &off, UHTTP_RESP_MAX, "\r\nX-PIOS-Handler: ");
#ifdef PIOS_HTTPD_NATIVE
        u_append(r, &off, UHTTP_RESP_MAX, "native-c");
#else
        u_append(r, &off, UHTTP_RESP_MAX, "PicoScript");
#endif
        u_append(r, &off, UHTTP_RESP_MAX, "\r\nContent-Length: ");
        u_append_u32(r, &off, UHTTP_RESP_MAX, blen);
        u_append(r, &off, UHTTP_RESP_MAX, "\r\nConnection: close\r\n\r\n");
        for (u32 i = 0; i < blen && off < UHTTP_RESP_MAX - 1U; i++)
            r[off++] = body[i];

        b->resp_len = off;
        b->resp_seq = cur;        /* signal the kernel: response ready */
        b->dbg_phase = 24U;       /* response published */
        uhttp_clean(b->resp, off);          /* response bytes to PoC first */
        uhttp_clean(&b->magic, UHTTP_LINE); /* then resp_len/resp_seq (Zone B) */
        last = cur;
    }
}
