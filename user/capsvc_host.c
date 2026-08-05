/*
 * capsvc_host.c - generic PicoScript capsule host process (EL0).
 *
 * One binary serves ANY capsvc-registered service: which service is decided
 * at compile time by CAPSVC_SVC_IDX (mirrors user/httpd.c's
 * UHTTP_BRIDGE_INDEX convention -- one flat binary, multiple launch sites).
 * This file has zero admin/web-specific knowledge; it only knows how to pump
 * the shared capsvc arena and run whatever PicoScript bytecode the kernel
 * preloaded for this service (capsvc.c's capsvc_preload_program(), sourced
 * from a WALFS/picowal capsule manifest card, or capsvc_admin_default_program
 * below if no card has been installed yet for this service's port).
 *
 * Constraint: image is mapped read-execute (W^X) -- no writable globals; all
 * mutable state lives in the shared arena, this process's own fixed-VA
 * scratch region (struct cap_scratch, see below), or the stack.
 *
 * BUILD REQUIREMENT: this file must be compiled with -fno-gcse (see
 * build_bootstrap.bat). This works around a confirmed GCC 13.3
 * (aarch64-none-elf) -fgcse miscompile: cap_process_slot() picks between two
 * `program` pointers (capsvc_admin_default_program, or a WALFS-loaded card in
 * sc->program_words) and passes whichever one into pv_vm_run(). At -O2 with
 * GCSE enabled, the compiler merged the two branches' tails and used the
 * WRONG register for `program` in the merged code, producing a near-NULL
 * pointer at runtime. pv_vm_run() then called pv_verify(), which faulted
 * dereferencing that pointer (EL0 permission fault, FAR≈8) -- silently, with
 * no fault ever logged, because this happens on the very first real request
 * after boot; the capsule process is simply dead afterward, so every
 * subsequent request just times out waiting for a reply that never comes.
 * Root-caused by: reproducing cleanly at -O0 but not -O2; bisecting across
 * all 43 flags -O2 adds over -O1 (binary search over -fno-* groups via
 * repeated QEMU boot+HTTP tests) to isolate -fno-gcse as sufficient and
 * necessary; and cross-checking with objdump, which showed the "loaded>0"
 * and "loaded==0" branches merging into a shared tail with a stale register
 * for the `program` argument. -fstrict-aliasing was ruled out first (tried
 * -fno-strict-aliasing alone; did not fix it) even though cap_pico_hook()'s
 * `(struct cap_pico_host *)ctx` reinterpret-cast is a real (separate,
 * currently harmless) aliasing violation worth keeping in mind if this ever
 * regresses on a different compiler version.
 *
 * The PicoScript integration mirrors user/httpd.c's picoweb_hook/pico_alloc_span
 * pattern, adapted to read the HTTP request from a capsvc_slot instead of a
 * uhttp_bridge, and to run once per request rather than being resident.
 */
#include "types.h"
#include "proc.h"
#include "capsvc.h"

/* Do NOT override PV_MAX_OUT (or any other picovm.h struct-shaping macro)
 * here: src/picovm.c is compiled as its own separate translation unit
 * (build_bootstrap.bat has no matching -DPV_MAX_OUT for picovm.o), so it
 * always sees picovm.h's plain default. A local override here would give
 * THIS file a different sizeof(pv_ctx)/field layout than the one picovm.c's
 * own compiled pv_init()/pv_vm_run()/pv_noop() use on the exact same
 * pointer -- an ABI mismatch that silently corrupts every field after the
 * shrunk one (including the ctx->host function pointer pv_noop() reads),
 * with no clean trap. This was found and fixed during this integration:
 * the mismatch made ctx->host read back as 0, so pv_noop() silently skipped
 * every hook call, and a corrupted call_stack sent PV_OP_RETURN into a
 * silent infinite loop. Response bodies are instead bounded to
 * CAPSVC_SLOT_DATA_MAX by simply truncating the copy out of ctx->out (see
 * cap_process_slot), independent of ctx->out's actual (larger) capacity. */
#include "picovm.h"
#include "pico_hooks.h"

#ifndef CAPSVC_SVC_IDX
#define CAPSVC_SVC_IDX 0U
#endif

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

#ifdef PIOS_USER_EL0
static inline u32 el0_getpid(void)
{
    register u64 x0 __asm__("x0");
    __asm__ volatile("svc #1" : "=r"(x0) :: "memory");
    return (u32)x0;
}
static inline void el0_wait_event(void)
{
    register u64 x0 __asm__("x0");
    __asm__ volatile("svc #4" : "=r"(x0) :: "memory");
}
#endif

static u32 cap_strlen(const char *s) { u32 n = 0; while (s && s[n]) n++; return n; }

static void cap_append(char *dst, u32 *off, u32 cap, const char *src)
{
    u32 n = cap_strlen(src);
    if (*off + n >= cap) n = (cap > *off) ? (cap - *off - 1) : 0;
    for (u32 i = 0; i < n; i++) dst[*off + i] = src[i];
    *off += n;
    dst[*off] = 0;
}

static void cap_append_u32(char *dst, u32 *off, u32 cap, u32 v)
{
    char tmp[12]; u32 i = 0;
    if (v == 0) tmp[i++] = '0';
    while (v) { tmp[i++] = (char)('0' + (v % 10)); v /= 10; }
    char rev[12];
    for (u32 j = 0; j < i; j++) rev[j] = tmp[i - 1 - j];
    rev[i] = 0;
    cap_append(dst, off, cap, rev);
}

/* Auto-generated by C:\source\picoscript\compile_admin_capsule.py from
 * picoscript_lang.py's real Compiler class (frozen PicoScript ISA -- opcodes
 * match include/pico_hooks.h exactly). Builds a small JSON reply that echoes
 * the real request path via Context.GetPath/Io.Write. This is only the
 * fallback used when no WALFS capsule manifest/card exists yet for this
 * service's port (capsvc_preload_program() in src/capsvc.c populates the
 * real program per service when a card is installed). 139 words, 556 bytes. */
static const u32 capsvc_admin_default_program[] = {
    0x000070E1u, 0x000080C8u, 0x51110001u, 0x4110007Bu, 0x00107072u, 0x51110001u,
    0x41100022u, 0x00107072u, 0x51110001u, 0x4110006Fu, 0x00107072u, 0x51110001u,
    0x4110006Bu, 0x00107072u, 0x51110001u, 0x41100022u, 0x00107072u, 0x51110001u,
    0x4110003Au, 0x00107072u, 0x51110001u, 0x41100074u, 0x00107072u, 0x51110001u,
    0x41100072u, 0x00107072u, 0x51110001u, 0x41100075u, 0x00107072u, 0x51110001u,
    0x41100065u, 0x00107072u, 0x51110001u, 0x4110002Cu, 0x00107072u, 0x51110001u,
    0x41100022u, 0x00107072u, 0x51110001u, 0x41100063u, 0x00107072u, 0x51110001u,
    0x41100061u, 0x00107072u, 0x51110001u, 0x41100070u, 0x00107072u, 0x51110001u,
    0x41100073u, 0x00107072u, 0x51110001u, 0x41100075u, 0x00107072u, 0x51110001u,
    0x4110006Cu, 0x00107072u, 0x51110001u, 0x41100065u, 0x00107072u, 0x51110001u,
    0x41100022u, 0x00107072u, 0x51110001u, 0x4110003Au, 0x00107072u, 0x51110001u,
    0x41100022u, 0x00107072u, 0x51110001u, 0x41100070u, 0x00107072u, 0x51110001u,
    0x41100069u, 0x00107072u, 0x51110001u, 0x41100063u, 0x00107072u, 0x51110001u,
    0x4110006Fu, 0x00107072u, 0x51110001u, 0x41100073u, 0x00107072u, 0x51110001u,
    0x41100063u, 0x00107072u, 0x51110001u, 0x41100072u, 0x00107072u, 0x51110001u,
    0x41100069u, 0x00107072u, 0x51110001u, 0x41100070u, 0x00107072u, 0x51110001u,
    0x41100074u, 0x00107072u, 0x51110001u, 0x41100022u, 0x00107072u, 0x51110001u,
    0x4110002Cu, 0x00107072u, 0x51110001u, 0x41100022u, 0x00107072u, 0x51110001u,
    0x41100070u, 0x00107072u, 0x51110001u, 0x41100061u, 0x00107072u, 0x51110001u,
    0x41100074u, 0x00107072u, 0x51110001u, 0x41100068u, 0x00107072u, 0x51110001u,
    0x41100022u, 0x00107072u, 0x51110001u, 0x4110003Au, 0x00107072u, 0x51110001u,
    0x41100022u, 0x00107072u, 0x00007071u, 0x51110001u, 0x41100022u, 0x00107072u,
    0x51110001u, 0x4110007Du, 0x00107072u, 0x51110001u, 0x4110000Au, 0x00107072u,
    0xC0000000u,
};

#define CAP_PICO_MEM_SIZE  4096U
#define CAP_PICO_MAX_SPANS 32U

struct cap_span {
    u32 ptr;
    u32 len;
};

/* Mirrors user/httpd.c's struct picoweb_host, adapted to a capsvc_slot
 * (per-request, not a resident bridge): the VM context must be the first
 * member so pv_host_fn's `ctx` pointer can be reinterpreted as this struct.
 *
 * pv_ctx (include/picovm.h) is genuinely large -- its card/span/writer/
 * capture tables add up to ~64KB by design (it's meant for a hosted or
 * heap-backed environment) -- measured via -Wframe-larger-than during this
 * integration: sizeof(struct cap_pico_host) is ~70KB. EL0 processes here get
 * a tiny fixed stack window (entry_sp = linked_base + 0x20000, only 128KB,
 * shared downward with nothing else in this single-threaded host), so this
 * struct must NEVER be stack-allocated -- doing so blew straight through the
 * ~57KB of real stack headroom left after the ~70KB loaded image and
 * corrupted the loaded code, producing an EC=0 "unknown reason" illegal-
 * instruction trap on the very first real request. Instead it lives at a
 * fixed VA within this process's OWN mapped 2MiB slot, past the loaded image
 * and its stack -- proc_exec_from_mem_el0()/mmu_user_table_build_split_el0_at()
 * already map and zero the ENTIRE slot (not just the loaded bytes), so this
 * is valid, private, RW, zero-initialized memory reachable with plain loads/
 * stores -- not a "writable global" in the W^X sense (no static/global
 * storage-class object exists; it is simply this process's own unused
 * mapped RAM, reinterpreted), and not a 2nd shared arena (only this process
 * ever touches it). */
#define CAP_IMAGE_LINK_BASE 0x2001000000ULL   /* must match user/httpd_el0.ld's fixed link VA */
#define CAP_HEAP_OFFSET     0x100000ULL       /* 1MiB in: comfortably past the ~70KB image + stack, comfortably inside the 2MiB slot */
#define CAP_HEAP_VA         (CAP_IMAGE_LINK_BASE + CAP_HEAP_OFFSET)

struct cap_pico_host {
    pv_ctx vm;
    struct capsvc_slot *slot;
    u32 req_len;
    u8 mem[CAP_PICO_MEM_SIZE];
    u32 arena_top;
    struct cap_span spans[CAP_PICO_MAX_SPANS];
    int span_count;
    int oom;
};

/* Bytecode word buffer also lives in the fixed scratch region, alongside
 * cap_pico_host, for the same stack-overflow-avoidance reason. */
struct cap_scratch {
    struct cap_pico_host host;
    u32 program_words[CAPSVC_PROG_MAX / 4U];
};

static int cap_alloc_span(struct cap_pico_host *h, const u8 *src, u32 n)
{
    if (h->span_count >= (int)CAP_PICO_MAX_SPANS) { h->oom = 1; return 0; }
    if (n > CAP_PICO_MEM_SIZE - h->arena_top) { h->oom = 1; n = CAP_PICO_MEM_SIZE - h->arena_top; }
    u32 ptr = h->arena_top;
    for (u32 i = 0; i < n; i++) h->mem[ptr + i] = src[i];
    h->arena_top += n;
    int handle = h->span_count++;
    h->spans[handle].ptr = ptr;
    h->spans[handle].len = n;
    return handle;
}

static void cap_write_span(struct cap_pico_host *h, i32 handle)
{
    if (handle <= 0 || handle >= h->span_count) return;
    struct cap_span s = h->spans[handle];
    for (u32 i = 0; i < s.len && h->vm.out_len < PV_MAX_OUT; i++)
        h->vm.out[h->vm.out_len++] = h->mem[s.ptr + i];
}

/* Request-line parsing mirrors user/httpd.c's req_bounds(), adapted to read
 * from a capsvc_slot's raw request bytes instead of a uhttp_bridge. */
static void cap_req_bounds(const u8 *req, u32 n, u32 *path_s, u32 *path_n)
{
    u32 i = 0;
    while (i < n && req[i] != ' ') i++;
    u32 ps = (i < n) ? i + 1U : n;
    u32 pe = ps;
    while (pe < n && req[pe] != ' ') pe++;
    for (u32 j = ps; j < pe; j++)
        if (req[j] == '?') { pe = j; break; }
    *path_s = ps;
    *path_n = (pe >= ps) ? pe - ps : 0;
}

static void cap_pico_hook(pv_ctx *ctx, int hook, int rd, int rs1, int rs2, int imm16)
{
    struct cap_pico_host *h = (struct cap_pico_host *)ctx;
    (void)imm16; (void)rs2;
    switch (hook) {
    case PV_HOOK_CONTEXT_GETPATH: {
        u32 ps, pn;
        cap_req_bounds(h->slot->data, h->req_len, &ps, &pn);
        ctx->regs[rd] = cap_alloc_span(h, h->slot->data + ps, pn);
        return;
    }
    case PV_HOOK_IO_WRITE:
        cap_write_span(h, ctx->regs[rs1]);
        return;
    case PV_HOOK_IO_WRITEBYTE:
        if (ctx->out_len < PV_MAX_OUT) ctx->out[ctx->out_len++] = (u8)(ctx->regs[rs1] & 0xFF);
        return;
    default:
        pv_default_host(ctx, hook, rd, rs1, rs2, imm16);
        return;
    }
}

/* Copy the kernel-preloaded per-service bytecode (capsvc.c's
 * capsvc_preload_program(), sourced from a WALFS capsule manifest card) out
 * of the shared NC arena into a private word buffer for pv_vm_run. Returns
 * word count, or 0 if no card was loaded (capsule falls back to
 * capsvc_admin_default_program). */
static u32 cap_load_program(struct capsvc_program *prog, u32 *words, u32 max_words)
{
    capsvc_inval(&prog->len, CAPSVC_LINE);
    u32 n = prog->len;
    if (n == 0 || n > CAPSVC_PROG_MAX || (n & 3U) != 0)
        return 0;
    capsvc_inval(prog->data, n);
    u32 nw = n / 4U;
    if (nw > max_words) nw = max_words;
    for (u32 i = 0; i < nw; i++)
        words[i] = (u32)prog->data[i * 4 + 0] |
                   ((u32)prog->data[i * 4 + 1] << 8) |
                   ((u32)prog->data[i * 4 + 2] << 16) |
                   ((u32)prog->data[i * 4 + 3] << 24);
    return nw;
}

static u32 cap_build_error_response(const char *reason, u8 *out, u32 cap)
{
    char body[128];
    u32 boff = 0;
    cap_append(body, &boff, sizeof(body), "{\"ok\":false,\"error\":\"");
    cap_append(body, &boff, sizeof(body), reason);
    cap_append(body, &boff, sizeof(body), "\"}\n");

    char *dst = (char *)out;
    u32 off = 0;
    cap_append(dst, &off, cap, "HTTP/1.0 500 Internal Server Error\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: ");
    cap_append_u32(dst, &off, cap, boff);
    cap_append(dst, &off, cap, "\r\n\r\n");
    cap_append(dst, &off, cap, body);
    return off;
}

static void cap_process_slot(struct capsvc_slot *slot, struct capsvc_program *prog)
{
    u32 req_len = slot->hdr.req_len;
    if (req_len > CAPSVC_SLOT_DATA_MAX) req_len = CAPSVC_SLOT_DATA_MAX;

    /* Fixed scratch region, not a stack local -- see struct cap_scratch's
     * comment. Single-threaded host: only one request is ever processed at
     * a time, so one shared scratch buffer at a well-known VA is safe. */
    struct cap_scratch *sc = (struct cap_scratch *)(usize)CAP_HEAP_VA;
    struct cap_pico_host *h = &sc->host;

    const u32 *program = capsvc_admin_default_program;
    u32 nwords = (u32)(sizeof(capsvc_admin_default_program) / sizeof(capsvc_admin_default_program[0]));
    u32 loaded = cap_load_program(prog, sc->program_words, CAPSVC_PROG_MAX / 4U);
    if (loaded > 0) {
        program = sc->program_words;
        nwords = loaded;
    }

    pv_init(&h->vm);
    h->vm.host = cap_pico_hook;
    h->vm.mem = h->mem;
    h->vm.mem_size = CAP_PICO_MEM_SIZE;
    h->slot = slot;
    h->req_len = req_len;
    h->arena_top = 0;
    h->span_count = 1;
    h->spans[0].ptr = 0;
    h->spans[0].len = 0;
    h->oom = 0;
    /* h->slot->data still holds the raw request bytes at this point (not yet
     * overwritten); Context.GetPath reads from it lazily during pv_vm_run. */
    (void)pv_vm_run(&h->vm, program, (int)nwords);

    u32 resp_len;
    if (h->vm.fault != PV_FAULT_NONE || h->oom) {
        resp_len = cap_build_error_response(h->oom ? "capsule out of memory" : "PicoScript fault",
                                             slot->data, CAPSVC_SLOT_DATA_MAX);
    } else {
        char *dst = (char *)slot->data;
        u32 off = 0;
        int status = (h->vm.http_status > 0) ? h->vm.http_status : 200;
        cap_append(dst, &off, CAPSVC_SLOT_DATA_MAX, "HTTP/1.0 ");
        cap_append_u32(dst, &off, CAPSVC_SLOT_DATA_MAX, (u32)status);
        cap_append(dst, &off, CAPSVC_SLOT_DATA_MAX, " OK\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: ");
        cap_append_u32(dst, &off, CAPSVC_SLOT_DATA_MAX, h->vm.out_len);
        cap_append(dst, &off, CAPSVC_SLOT_DATA_MAX, "\r\n\r\n");
        for (u32 i = 0; i < (u32)h->vm.out_len && off < CAPSVC_SLOT_DATA_MAX; i++)
            dst[off++] = (char)h->vm.out[i];
        resp_len = off;
    }

    capsvc_clean(slot->data, resp_len);
    slot->hdr.resp_len = resp_len;
    slot->hdr.state = CAPSVC_SLOT_REPLY;
    capsvc_clean(&slot->hdr, CAPSVC_LINE);
}

void user_main(struct kernel_api *api)
{
    struct capsvc_arena *a = capsvc_arena();
#ifdef PIOS_USER_EL0
    (void)api;
    u32 pid = el0_getpid();
#else
    u32 pid = api->getpid();
#endif
    a->attach[CAPSVC_SVC_IDX].pid = pid;
    a->attach[CAPSVC_SVC_IDX].svc_idx = CAPSVC_SVC_IDX;
    capsvc_clean(&a->attach[CAPSVC_SVC_IDX], CAPSVC_LINE);
    a->attach[CAPSVC_SVC_IDX].magic = CAPSVC_ATTACH_MAGIC;
    capsvc_clean(&a->attach[CAPSVC_SVC_IDX], CAPSVC_LINE);

    /* Write-once description, own dedicated line -- a plain string compiled
     * into THIS binary, read by the kernel-side dashboard/"services" command.
     * Never hardcoded by the launcher: it genuinely lives in this .c file,
     * and (unlike a per-binary #ifdef) is the SAME generic text regardless of
     * which service/port this capsule instance ends up serving, since this
     * host is deliberately generic (see the file header comment). */
    {
        const char *desc = "generic PicoScript capsule host (capsvc)";
        u32 n = cap_strlen(desc);
        if (n >= CAPSVC_DESC_MAX) n = CAPSVC_DESC_MAX - 1U;
        for (u32 i = 0; i < n; i++) a->attach[CAPSVC_SVC_IDX].description[i] = desc[i];
        a->attach[CAPSVC_SVC_IDX].description[n] = 0;
        capsvc_clean(a->attach[CAPSVC_SVC_IDX].description, CAPSVC_LINE);
    }

    for (;;) {
        bool did_work = false;
        for (u32 i = 0; i < CAPSVC_SLOT_COUNT; i++) {
            struct capsvc_slot *slot = &a->slots[i];
            capsvc_inval(&slot->hdr, CAPSVC_LINE);
            if (slot->hdr.state != CAPSVC_SLOT_REQUEST || slot->hdr.svc_idx != CAPSVC_SVC_IDX)
                continue;
            capsvc_inval(slot->data, slot->hdr.req_len);
            cap_process_slot(slot, &a->programs[CAPSVC_SVC_IDX]);
            did_work = true;
        }
        if (!did_work) {
#ifdef PIOS_USER_EL0
            el0_wait_event();   /* kernel wake path (proc_post_remote_wake) sends SEV */
#else
            api->park();
#endif
        }
    }
}
