/* proc.c - Cooperative process manager for user cores (2-3) */

#include "proc.h"
#include "core.h"
#include "core_env.h"
#include "walfs.h"
#include "principal.h"
#include "el2.h"
#include "uart.h"
#include "timer.h"
#include "socket.h"
#include "dma.h"
#include "simd.h"
#include "fb.h"
#include "usb_kbd.h"
#include "dns.h"
#include "tensor.h"
#include "v3d.h"
#include "fifo.h"
#include "mmu.h"
#include "ipc_queue.h"
#include "ipc_stream.h"
#include "ipc_proc.h"
#include "pipe.h"
#include "exception.h"
#include "ksem.h"
#include "workq.h"
#include "picowal_db.h"
#include "pix.h"

static struct process  procs[MAX_PROCS_PER_CORE];
static struct proc_context scheduler_ctx;
static u32 current_proc;   /* index into procs[] */
static u32 rr_cursor;
static u32 next_pid;
static bool initialized;
static u64 heap_top[MAX_PROCS_PER_CORE];
static volatile bool preempt_enabled[3];
static volatile bool preempt_armed[3];
static volatile bool preempt_pending[3];
static u64 preempt_quantum_ticks[3];
static u64 preempt_count_core[3];
static struct proc_security_stats proc_sec_stats;

#define PROC_LAUNCH_PATH_MAX 96
struct proc_launch_req {
    volatile u32 pending;
    i32 result_pid;
    u32 principal_id;
    u32 has_principal;
    u32 priority_class;
    u32 has_priority;
    u32 migrate_keep_pid;
    u32 migrate_pid;
    u32 migrate_parent_pid;
    u64 migrate_runtime_ticks;
    u32 migrate_preemptions;
    u32 migrate_quota_mem_kib;
    u32 migrate_quota_cpu_ms;
    u32 migrate_quota_ipc_objs;
    u32 migrate_quota_fs_write_kib;
    u32 migrate_usage_ipc_objs;
    u64 migrate_usage_fs_write_bytes;
    u32 migrate_heap_used;
    u32 migrate_exec_image_size;
    u32 migrate_exec_hash_baseline;
    u32 migrate_exec_hash_last;
    u64 migrate_exec_hash_next_check_tick;
    u32 migrate_exec_hash_check_nonce;
    u32 has_migrate_state;
    char path[PROC_LAUNCH_PATH_MAX];
};
static struct proc_launch_req launch_req[3];
static inline bool on_user_core(void);
static inline u32 user_core_slot(void);
static i32 proc_exec_with_policy(const char *path, u32 priority_class, u32 affinity_core);
static void proc_account_runtime(struct process *p);

extern u8 __text_start;
extern u8 __text_end;
static u32 proc_el1_integrity_baseline;
static u64 proc_el1_integrity_next_check_tick;

#define MAX_PAGED_IO_HANDLES 16
struct paged_io_handle {
    bool used;
    u32 owner_pid;
    u32 page_size;
    u32 flags;
    u64 inode_id;
};
static struct paged_io_handle paged_io_tab[MAX_PAGED_IO_HANDLES];

static bool proc_prio_valid(u32 p)
{
    return p <= PROC_PRIO_REALTIME;
}

static i32 proc_find_slot_by_pid(u32 pid)
{
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        if (procs[i].pid == pid)
            return (i32)i;
    }
    return -1;
}

static u64 proc_integrity_next_tick(u32 pid, u32 nonce)
{
    u64 now = timer_ticks();
    u64 seed = now ^ ((u64)pid << 21) ^ ((u64)nonce << 7) ^ ((u64)core_id() << 3);
    u32 h = hw_crc32c(&seed, sizeof(seed));
    u64 span = 128ULL + (u64)(h & 0x1FFU); /* randomized 128..639 ticks */
    return now + span;
}

static u32 proc_el1_integrity_hash_now(void)
{
    u64 s = (u64)(usize)&__text_start;
    u64 e = (u64)(usize)&__text_end;
    if (e <= s) return 0;
    return hw_crc32c((const void *)(usize)s, (u32)(e - s));
}

static void proc_kill_capsule_members(const struct process *src, u32 exit_code)
{
    if (!src) return;
    proc_sec_stats.capsule_kills++;
    bool cap = src->capsule_enabled;
    u32 hash = src->capsule_manifest_hash;
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        struct process *p = &procs[i];
        if (!(p->state == PROC_READY || p->state == PROC_RUNNING || p->state == PROC_BLOCKED))
            continue;
        if (!cap) {
            if (p->pid != src->pid) continue;
        } else {
            if (!p->capsule_enabled || p->capsule_manifest_hash != hash) continue;
        }
        if (p->state == PROC_RUNNING)
            proc_account_runtime(p);
        p->state = PROC_DEAD;
        p->exit_code = exit_code;
    }
}

static bool proc_integrity_maybe_check(u32 slot)
{
    if (slot >= MAX_PROCS_PER_CORE) return false;
    struct process *p = &procs[slot];
    if (p->exec_image_size == 0 || p->exec_hash_baseline == 0)
        return true;
    u64 now = timer_ticks();
    if (proc_el1_integrity_baseline != 0 && now >= proc_el1_integrity_next_check_tick) {
        u32 h = proc_el1_integrity_hash_now();
        if (h != proc_el1_integrity_baseline)
            exception_pisod("EL1 integrity failure", 4, 0x3D, 0, 0, 0);
        proc_el1_integrity_next_check_tick = now + 512ULL + (u64)((h ^ p->pid) & 0x1FFU);
    }
    if (now < p->exec_hash_next_check_tick)
        return true;
    proc_sec_stats.integrity_checks++;
    u64 out = ~0ULL;
    if (el2_hvc_call(EL2_HVC_INTEGRITY_CHECK,
                     (u64)(usize)p->base,
                     (u64)p->exec_image_size,
                     (u64)p->exec_hash_baseline,
                     (u64)p->exec_hash_check_nonce,
                     &out) != 0) {
        proc_sec_stats.integrity_failures++;
        proc_kill_capsule_members(p, 0xFFFF0007U);
        return false;
    }
    if ((u32)out == EL2_INTEGRITY_EL2_CHANGED) {
        proc_sec_stats.integrity_failures++;
        exception_pisod("EL2 integrity failure", 3, 0x3E, 0, 0, 0);
    }
    if ((u32)out == EL2_INTEGRITY_EL1_CHANGED) {
        proc_sec_stats.integrity_failures++;
        exception_pisod("EL1 integrity failure", 4, 0x3D, 0, 0, 0);
    }
    if (out != 0ULL) {
        proc_sec_stats.integrity_failures++;
        proc_kill_capsule_members(p, 0xFFFF0007U);
        return false;
    }
    p->exec_hash_last = p->exec_hash_baseline;
    p->exec_hash_check_nonce++;
    p->exec_hash_next_check_tick = proc_integrity_next_tick(p->pid, p->exec_hash_check_nonce);
    return true;
}

static u64 proc_quantum_for_prio(u32 p)
{
    if (p == PROC_PRIO_REALTIME) return 1;
    if (p == PROC_PRIO_HIGH) return 2;
    if (p == PROC_PRIO_LOW) return 10;
    if (p == PROC_PRIO_LAZY) return 20;
    return 5; /* normal */
}

static void proc_account_runtime(struct process *p)
{
    if (!p || p->state != PROC_RUNNING)
        return;
    u64 now = timer_ticks();
    if (now >= p->ticks)
        p->runtime_ticks += (now - p->ticks);
    p->ticks = now;
    if (p->capsule_enabled && p->quota_cpu_ms > 0 && p->runtime_ticks > (u64)p->quota_cpu_ms) {
        p->state = PROC_DEAD;
        p->exit_code = 0xFFFF0006U;
    }
}

static void proc_handle_launch_request(void)
{
    if (!on_user_core())
        return;
    u32 uc = user_core_slot();
    if (!launch_req[uc].pending)
        return;

    u32 prev_principal = principal_current();
    if (launch_req[uc].has_principal)
        principal_set_current(launch_req[uc].principal_id);
    u32 prio = launch_req[uc].has_priority ? launch_req[uc].priority_class : PROC_PRIO_NORMAL;
    i32 pid = proc_exec_with_policy(launch_req[uc].path, prio, core_id());
    if (pid > 0 && launch_req[uc].has_migrate_state) {
        i32 slot = proc_find_slot_by_pid((u32)pid);
        if (slot >= 0) {
            struct process *p = &procs[(u32)slot];
            if (launch_req[uc].migrate_keep_pid)
                p->pid = launch_req[uc].migrate_pid;
            p->parent_pid = launch_req[uc].migrate_parent_pid;
            p->runtime_ticks = launch_req[uc].migrate_runtime_ticks;
            p->preemptions = launch_req[uc].migrate_preemptions;
            p->quota_mem_kib = launch_req[uc].migrate_quota_mem_kib;
            p->quota_cpu_ms = launch_req[uc].migrate_quota_cpu_ms;
            p->quota_ipc_objs = launch_req[uc].migrate_quota_ipc_objs;
            p->quota_fs_write_kib = launch_req[uc].migrate_quota_fs_write_kib;
            p->usage_ipc_objs = launch_req[uc].migrate_usage_ipc_objs;
            p->usage_fs_write_bytes = launch_req[uc].migrate_usage_fs_write_bytes;
            p->exec_image_size = launch_req[uc].migrate_exec_image_size;
            p->exec_hash_baseline = launch_req[uc].migrate_exec_hash_baseline;
            p->exec_hash_last = launch_req[uc].migrate_exec_hash_last;
            p->exec_hash_next_check_tick = launch_req[uc].migrate_exec_hash_next_check_tick;
            p->exec_hash_check_nonce = launch_req[uc].migrate_exec_hash_check_nonce;
            if (launch_req[uc].migrate_heap_used > 0) {
                u64 cur = heap_top[(u32)slot];
                u64 want = (u64)(usize)p->base + launch_req[uc].migrate_heap_used;
                u64 lim = (u64)(usize)p->base + p->mem_size - 65536UL;
                if (want > cur && want < lim)
                    heap_top[(u32)slot] = want;
            }
            pid = (i32)p->pid;
        } else {
            pid = -1;
        }
    }
    if (launch_req[uc].has_principal)
        principal_set_current(prev_principal);
    launch_req[uc].result_pid = pid;
    dmb();
    launch_req[uc].pending = 0;
    sev();
}

static inline bool on_user_core(void)
{
    u32 c = core_id();
    return c == CORE_USERM || c == CORE_USER0 || c == CORE_USER1;
}

static inline u32 user_core_slot(void)
{
    return core_id() - CORE_USERM;
}

/* Validate a user pointer is within the current process's memory slot */
static bool ptr_valid(const void *ptr, u32 len) {
    struct process *p = &procs[current_proc];
    u64 addr = (u64)(usize)ptr;
    u64 end = addr + len;
    u64 slot_start = (u64)(usize)p->base;
    u64 slot_end = slot_start + p->mem_size;
    return addr >= slot_start && end <= slot_end && end >= addr;
}

static bool ptr_valid_cstr(const char *s, u32 max_len)
{
    if (!s || max_len == 0) return false;
    for (u32 i = 0; i < max_len; i++) {
        if (!ptr_valid(s + i, 1)) return false;
        if (s[i] == 0) return i != 0;
    }
    return false;
}

/* Send a FIFO request to the disk core and block until reply */
static void fs_request(struct fifo_msg *msg, struct fifo_msg *reply)
{
    fifo_push(core_id(), CORE_DISK, msg);
    while (!fifo_pop(core_id(), CORE_DISK, reply))
        wfe();
}

/* OWASP A01: capability gate — check before privileged operations */
static bool has_cap(u32 cap) {
    return principal_has_cap(principal_current(), cap);
}

static bool has_disk_cap(void) { return has_cap(PRINCIPAL_DISK); }
static bool has_net_cap(void)  { return has_cap(PRINCIPAL_NET); }
static bool has_ipc_cap(void)  { return has_cap(PRINCIPAL_IPC); }

static bool proc_is_active_state(u32 state)
{
    return state == PROC_READY || state == PROC_RUNNING || state == PROC_BLOCKED;
}

static bool str_eq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        a++; b++;
    }
    return *a == 0 && *b == 0;
}

static bool starts_with(const char *s, const char *pfx)
{
    if (!s || !pfx) return false;
    while (*pfx) {
        if (*s++ != *pfx++) return false;
    }
    return true;
}

static u32 u32_parse10(const char *s, bool *ok)
{
    u32 v = 0;
    bool have = false;
    if (!s) { if (ok) *ok = false; return 0; }
    while (*s) {
        char c = *s++;
        if (c < '0' || c > '9') { if (ok) *ok = false; return 0; }
        have = true;
        v = v * 10U + (u32)(c - '0');
    }
    if (ok) *ok = have;
    return v;
}

static void copy_trim(char *dst, u32 dst_max, const char *src, u32 n)
{
    if (!dst || dst_max == 0) return;
    while (n > 0 && (*src == ' ' || *src == '\t')) { src++; n--; }
    while (n > 0 && (src[n - 1] == ' ' || src[n - 1] == '\t')) n--;
    u32 p = 0;
    while (p + 1 < dst_max && p < n) { dst[p] = src[p]; p++; }
    dst[p] = 0;
}

static bool capsule_allows_fs_path(const struct process *p, const char *path)
{
    if (!p || !path) return false;
    if (!p->capsule_enabled || p->capsule_fs_prefix_count == 0) return true;
    for (u32 i = 0; i < p->capsule_fs_prefix_count; i++) {
        if (starts_with(path, p->capsule_fs_prefix[i])) return true;
    }
    return false;
}

static bool capsule_allows_ipc_name(const struct process *p, const char *name)
{
    if (!p || !name) return false;
    if (!p->capsule_enabled || p->capsule_ipc_prefix_count == 0) return true;
    for (u32 i = 0; i < p->capsule_ipc_prefix_count; i++) {
        if (starts_with(name, p->capsule_ipc_prefix[i])) return true;
    }
    return false;
}

static bool capsule_allows_pipe_path(const struct process *p, const char *path)
{
    if (!p || !path) return false;
    if (!p->capsule_enabled || p->capsule_pipe_prefix_count == 0) return true;
    for (u32 i = 0; i < p->capsule_pipe_prefix_count; i++) {
        if (starts_with(path, p->capsule_pipe_prefix[i])) return true;
    }
    return false;
}

static bool capsule_allows_card(const struct process *p, u16 card)
{
    if (!p) return false;
    if (!p->capsule_enabled || p->capsule_card_range_count == 0) return true;
    for (u32 i = 0; i < p->capsule_card_range_count; i++) {
        if (card >= p->capsule_card_ranges[i].lo && card <= p->capsule_card_ranges[i].hi) return true;
    }
    return false;
}

static bool capsule_allows_port(const struct process *p, u16 port)
{
    if (!p) return false;
    if (!p->capsule_enabled || p->capsule_port_range_count == 0) return true;
    for (u32 i = 0; i < p->capsule_port_range_count; i++) {
        if (port >= p->capsule_port_ranges[i].lo && port <= p->capsule_port_ranges[i].hi) return true;
    }
    return false;
}

static bool capsule_resolve_fs_path(const struct process *p, const char *in, char *out, u32 out_max)
{
    if (!p || !in || !out || out_max < 2)
        return false;
    if (!p->capsule_enabled || p->capsule_vfs_root[0] == 0) {
        u32 n = pios_strlen(in);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i <= n; i++) out[i] = in[i];
        return true;
    }

    u32 rlen = pios_strlen(p->capsule_vfs_root);
    u32 ilen = pios_strlen(in);
    for (u32 i = 0; i + 1 < ilen; i++) {
        if (in[i] == '.' && in[i + 1] == '.' &&
            (i == 0 || in[i - 1] == '/') &&
            (i + 2 == ilen || in[i + 2] == '/'))
            return false;
    }
    bool abs = (in[0] == '/');
    u32 need = rlen + (abs ? 0 : 1) + ilen + 1;
    if (need > out_max) return false;

    u32 p0 = 0;
    for (u32 i = 0; i < rlen; i++) out[p0++] = p->capsule_vfs_root[i];
    if (!abs) out[p0++] = '/';
    for (u32 i = 0; i < ilen; i++) out[p0++] = in[i];
    out[p0] = 0;
    return true;
}

static bool capsule_quota_ipc_consume(struct process *p)
{
    if (!p) return false;
    if (!p->capsule_enabled || p->quota_ipc_objs == 0) {
        p->usage_ipc_objs++;
        return true;
    }
    if (p->usage_ipc_objs >= p->quota_ipc_objs)
        return false;
    p->usage_ipc_objs++;
    return true;
}

static bool capsule_quota_fs_write_allow(struct process *p, u32 write_len)
{
    if (!p) return false;
    if (!p->capsule_enabled || p->quota_fs_write_kib == 0) return true;
    u64 lim = (u64)p->quota_fs_write_kib * 1024ULL;
    return (p->usage_fs_write_bytes + write_len <= lim);
}

static void capsule_quota_fs_write_account(struct process *p, u32 write_len)
{
    if (!p) return;
    p->usage_fs_write_bytes += write_len;
}

static void capsule_manifest_defaults(struct process *p)
{
    p->capsule_enabled = true;
    p->capsule_manifest_hash = 0;
    p->capsule_allow_spawn = true;
    p->capsule_allow_wait = true;
    p->capsule_allow_nprocs = true;
    p->capsule_group[0] = 0;
    p->capsule_vfs_root[0] = 0;
    p->quota_mem_kib = 0;
    p->quota_cpu_ms = 0;
    p->quota_ipc_objs = 0;
    p->quota_fs_write_kib = 0;
    p->usage_ipc_objs = 0;
    p->usage_fs_write_bytes = 0;
    p->capsule_fs_prefix_count = 0;
    p->capsule_ipc_prefix_count = 0;
    p->capsule_pipe_prefix_count = 0;
    p->capsule_card_range_count = 0;
    p->capsule_port_range_count = 0;
}

static bool capsule_str_has_path_escape(const char *s)
{
    if (!s) return true;
    for (u32 i = 0; s[i]; i++) {
        if ((s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n'))
            return true;
        if (s[i] == '.' && s[i + 1] == '.' &&
            (i == 0 || s[i - 1] == '/') &&
            (s[i + 2] == 0 || s[i + 2] == '/'))
            return true;
    }
    return false;
}

static bool capsule_add_prefixes_raw(char *dst, u32 stride, u32 *count, u32 max_count, u32 max_len,
                                     const char *csv, bool require_abs_path)
{
    if (!dst || !count || !csv || stride == 0) return false;
    const char *p = csv;
    while (*p && *count < max_count) {
        const char *start = p;
        while (*p && *p != ',' && *p != ';') p++;
        char tmp[64];
        copy_trim(tmp, sizeof(tmp), start, (u32)(p - start));
        if (tmp[0]) {
            if (require_abs_path && tmp[0] != '/')
                return false;
            if (capsule_str_has_path_escape(tmp))
                return false;
            char *slot = dst + ((u64)(*count) * stride);
            u32 i = 0;
            while (tmp[i] && i + 1 < max_len) { slot[i] = tmp[i]; i++; }
            slot[i] = 0;
            (*count)++;
        }
        if (*p == ',' || *p == ';') p++;
    }
    if (*p != 0)
        return false;
    return true;
}

static bool capsule_add_card_ranges(struct process *pr, const char *csv)
{
    if (!pr || !csv) return false;
    const char *p = csv;
    while (*p && pr->capsule_card_range_count < 8) {
        const char *start = p;
        while (*p && *p != ',' && *p != ';') p++;
        char tok[32];
        copy_trim(tok, sizeof(tok), start, (u32)(p - start));
        if (tok[0]) {
            u16 lo = 0, hi = 0;
            bool ok = false;
            char *dash = NULL;
            for (u32 i = 0; tok[i]; i++) if (tok[i] == '-') { dash = &tok[i]; break; }
            if (!dash) {
                u32 v = u32_parse10(tok, &ok);
                if (ok && v <= PICOWAL_CARD_MAX) { lo = (u16)v; hi = (u16)v; }
                else ok = false;
            } else {
                *dash = 0;
                u32 a = u32_parse10(tok, &ok);
                if (ok) {
                    bool ok2 = false;
                    u32 b = u32_parse10(dash + 1, &ok2);
                    ok = ok2;
                    if (ok && a <= PICOWAL_CARD_MAX && b <= PICOWAL_CARD_MAX && a <= b) {
                        lo = (u16)a; hi = (u16)b;
                    } else ok = false;
                }
            }
            if (ok) {
                u32 i = pr->capsule_card_range_count++;
                pr->capsule_card_ranges[i].lo = lo;
                pr->capsule_card_ranges[i].hi = hi;
            } else return false;
        }
        if (*p == ',' || *p == ';') p++;
    }
    if (*p != 0)
        return false;
    return true;
}

static bool capsule_add_port_ranges(struct process *pr, const char *csv)
{
    if (!pr || !csv) return false;
    const char *p = csv;
    while (*p && pr->capsule_port_range_count < 8) {
        const char *start = p;
        while (*p && *p != ',' && *p != ';') p++;
        char tok[32];
        copy_trim(tok, sizeof(tok), start, (u32)(p - start));
        if (tok[0]) {
            u16 lo = 0, hi = 0;
            bool ok = false;
            char *dash = NULL;
            for (u32 i = 0; tok[i]; i++) if (tok[i] == '-') { dash = &tok[i]; break; }
            if (!dash) {
                u32 v = u32_parse10(tok, &ok);
                if (ok && v <= 65535U) { lo = (u16)v; hi = (u16)v; } else ok = false;
            } else {
                *dash = 0;
                u32 a = u32_parse10(tok, &ok);
                if (ok) {
                    bool ok2 = false;
                    u32 b = u32_parse10(dash + 1, &ok2);
                    ok = ok2;
                    if (ok && a <= 65535U && b <= 65535U && a <= b) {
                        lo = (u16)a; hi = (u16)b;
                    } else ok = false;
                }
            }
            if (ok) {
                u32 i = pr->capsule_port_range_count++;
                pr->capsule_port_ranges[i].lo = lo;
                pr->capsule_port_ranges[i].hi = hi;
            } else return false;
        }
        if (*p == ',' || *p == ';') p++;
    }
    if (*p != 0)
        return false;
    return true;
}

static bool capsule_manifest_load(struct process *p, const char *path)
{
    if (!p || !path) return false;
    capsule_manifest_defaults(p);
    p->capsule_manifest_hash = hw_crc32c(path, pios_strlen(path));
    char mp[256];
    u32 pl = pios_strlen(path);
    if (pl + 5 >= sizeof(mp)) return false;
    for (u32 i = 0; i < pl; i++) mp[i] = path[i];
    mp[pl + 0] = '.';
    mp[pl + 1] = 'c';
    mp[pl + 2] = 'a';
    mp[pl + 3] = 'p';
    mp[pl + 4] = 0;

    u64 id = walfs_find(mp);
    if (!id) return true;
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) return false;
    char buf[1024];
    u32 n = (u32)((ino.size > sizeof(buf) - 1) ? (sizeof(buf) - 1) : ino.size);
    n = walfs_read(id, 0, buf, n);
    buf[n] = 0;
    p->capsule_manifest_hash = hw_crc32c(buf, n);
    p->capsule_enabled = true;

    u32 i = 0;
    while (i < n) {
        u32 ls = i;
        while (i < n && buf[i] != '\n' && buf[i] != '\r') i++;
        u32 le = i;
        while (i < n && (buf[i] == '\n' || buf[i] == '\r')) i++;
        if (le <= ls) continue;
        char line[160];
        copy_trim(line, sizeof(line), &buf[ls], le - ls);
        if (!line[0] || line[0] == '#') continue;
        char *eq = NULL;
        for (u32 j = 0; line[j]; j++) if (line[j] == '=') { eq = &line[j]; break; }
        if (!eq) return false;
        *eq = 0;
        char key[32], val[128];
        copy_trim(key, sizeof(key), line, pios_strlen(line));
        copy_trim(val, sizeof(val), eq + 1, pios_strlen(eq + 1));
        if (str_eq(key, "capsule")) {
            if (!(str_eq(val, "on") || str_eq(val, "true") || str_eq(val, "1") ||
                  str_eq(val, "off") || str_eq(val, "false") || str_eq(val, "0")))
                return false;
            p->capsule_enabled = str_eq(val, "on") || str_eq(val, "true") || str_eq(val, "1");
        } else if (str_eq(key, "spawn")) {
            if (!(str_eq(val, "allow") || str_eq(val, "true") || str_eq(val, "1") ||
                  str_eq(val, "deny") || str_eq(val, "false") || str_eq(val, "0")))
                return false;
            p->capsule_allow_spawn = str_eq(val, "allow") || str_eq(val, "true") || str_eq(val, "1");
        } else if (str_eq(key, "wait")) {
            if (!(str_eq(val, "allow") || str_eq(val, "true") || str_eq(val, "1") ||
                  str_eq(val, "deny") || str_eq(val, "false") || str_eq(val, "0")))
                return false;
            p->capsule_allow_wait = str_eq(val, "allow") || str_eq(val, "true") || str_eq(val, "1");
        } else if (str_eq(key, "nprocs")) {
            if (!(str_eq(val, "allow") || str_eq(val, "true") || str_eq(val, "1") ||
                  str_eq(val, "deny") || str_eq(val, "false") || str_eq(val, "0")))
                return false;
            p->capsule_allow_nprocs = str_eq(val, "allow") || str_eq(val, "true") || str_eq(val, "1");
        } else if (str_eq(key, "group")) {
            if (capsule_str_has_path_escape(val))
                return false;
            copy_trim(p->capsule_group, sizeof(p->capsule_group), val, pios_strlen(val));
        }
        else if (str_eq(key, "mem_kib")) {
            bool ok = false; u32 v = u32_parse10(val, &ok); if (!ok || v > (PROC_SLOT_SIZE >> 10)) return false; p->quota_mem_kib = v;
        } else if (str_eq(key, "cpu_ms")) {
            bool ok = false; u32 v = u32_parse10(val, &ok); if (!ok) return false; p->quota_cpu_ms = v;
        } else if (str_eq(key, "ipc_objs")) {
            bool ok = false; u32 v = u32_parse10(val, &ok); if (!ok) return false; p->quota_ipc_objs = v;
        } else if (str_eq(key, "fs_write_kib")) {
            bool ok = false; u32 v = u32_parse10(val, &ok); if (!ok) return false; p->quota_fs_write_kib = v;
        }
        else if (str_eq(key, "vfs")) {
            copy_trim(p->capsule_vfs_root, sizeof(p->capsule_vfs_root), val, pios_strlen(val));
            if (p->capsule_vfs_root[0] != '/' || capsule_str_has_path_escape(p->capsule_vfs_root))
                return false;
        }
        else if (str_eq(key, "fs")) {
            if (!capsule_add_prefixes_raw((char *)p->capsule_fs_prefix, 64, &p->capsule_fs_prefix_count, 8, 64, val, true))
                return false;
        } else if (str_eq(key, "ipc")) {
            if (!capsule_add_prefixes_raw((char *)p->capsule_ipc_prefix, 32, &p->capsule_ipc_prefix_count, 8, 32, val, false))
                return false;
        } else if (str_eq(key, "pipe")) {
            if (!capsule_add_prefixes_raw((char *)p->capsule_pipe_prefix, 64, &p->capsule_pipe_prefix_count, 8, 64, val, true))
                return false;
        } else if (str_eq(key, "cards")) {
            if (!capsule_add_card_ranges(p, val))
                return false;
        } else if (str_eq(key, "ports")) {
            if (!capsule_add_port_ranges(p, val))
                return false;
        } else return false;
    }
    if (p->capsule_group[0])
        p->capsule_manifest_hash = hw_crc32c(p->capsule_group, pios_strlen(p->capsule_group));
    return true;
}

/* ---- Forward declarations ---- */
static i32   sys_yield(void);
static i32   sys_exit(u32 code);
static u32   sys_getpid(void);
static void  sys_print(const char *msg);
static void  sys_putc(char c);
static i32   sys_getc(void);
static i32   sys_try_getc(void);
static u64   sys_ticks(void);
static void  sys_sleep_ms(u64 ms);
static void  sys_sleep_us(u64 us);
static u64   sys_runtime_ms(void);
static u64   sys_monotonic_ms(void);
static u64   sys_utc_ms(void);
static i32   sys_set_utc_ms(u64 utc_ms);
static u64   sys_rtc_ms(void);
static i32   sys_set_tz_offset_min(i32 offset_min);
static i32   sys_get_tz_offset_min(void);
static i32   sys_list_tz_offsets(i32 *out_offsets, u32 max_entries);
static i32   sys_open(const char *path, u32 flags);
static i32   sys_creat(const char *path, u32 flags, u32 mode);
static i32   sys_read(i32 fd, void *buf, u32 len);
static i32   sys_write(i32 fd, const void *buf, u32 len);
static i32   sys_pread(i32 fd, void *buf, u32 len, u64 offset);
static i32   sys_pwrite(i32 fd, const void *buf, u32 len, u64 offset);
static i32   sys_close(i32 fd);
static i32   sys_stat(const char *path, void *out);
static i32   sys_mkdir(const char *path);
static i32   sys_unlink(const char *path);
static i32   sys_readdir(const char *path, void *entries, u32 max_entries);
static i32   sys_page_open(const char *path, u32 page_size, u32 flags);
static i32   sys_page_read(i32 page_id, u64 page_idx, void *out_page, u32 out_len);
static i32   sys_page_write(i32 page_id, u64 page_idx, const void *in_page, u32 in_len);
static i32   sys_page_flush(i32 page_id);
static i32   sys_page_stat(i32 page_id, struct paged_io_stat *out);
static i32   sys_page_close(i32 page_id);
static void  sys_fb_putc(char c);
static void  sys_fb_print(const char *s);
static void  sys_fb_color(u32 fg, u32 bg);
static void  sys_fb_clear(u32 color);
static void  sys_fb_pixel(u32 x, u32 y, u32 color);
static i32   sys_socket(u32 type);
static i32   sys_bind(i32 fd, u32 ip, u16 port);
static i32   sys_connect(i32 fd, u32 ip, u16 port);
static i32   sys_listen(i32 fd, u32 backlog);
static i32   sys_accept(i32 fd, u32 *client_ip, u16 *client_port);
static i32   sys_send(i32 fd, const void *data, u32 len);
static i32   sys_recv(i32 fd, void *buf, u32 len);
static i32   sys_sendto(i32 fd, const void *data, u32 len, u32 ip, u16 port);
static i32   sys_recvfrom(i32 fd, void *buf, u32 len, u32 *src_ip, u16 *src_port);
static i32   sys_sock_close(i32 fd);
static i32   sys_resolve(const char *hostname, u32 *ip_out);
static u32   sys_whoami(void);
static i32   sys_auth(const char *user, const char *pass);
static void *sys_sbrk(i32 increment);
static void *sys_memset(void *dst, i32 c, u32 n);
static void *sys_memcpy(void *dst, const void *src, u32 n);
static u32   sys_strlen(const char *s);
static i32   sys_spawn(const char *path);
static i32   sys_wait(i32 pid);
static u32   sys_nprocs(void);
static i32   sys_sem_create(u32 initial);
static i32   sys_sem_wait(i32 id);
static i32   sys_sem_post(i32 id);
static i32   sys_lock_create(void);
static i32   sys_lock_acquire(i32 id);
static i32   sys_lock_release(i32 id);
static i32   sys_kv_put(u32 key, const void *data, u32 len);
static i32   sys_kv_get(u32 key, void *out, u32 out_len);
static i32   sys_kv_del(u32 key);
static i32   sys_kv_list(u16 card, u32 *out_keys, u32 max_keys);
static i32   sys_event_emit(u32 type, const void *data, u32 len);
static i32   sys_event_next(struct appf_event_record *out);
static i32   sys_log_write(u32 level, const char *msg, u32 len);
static i32   sys_log_next(struct appf_log_record *out);
static i32   sys_svc_register(const char *name, u32 kind, u32 endpoint, u32 flags);
static i32   sys_svc_resolve(const char *name, struct appf_service_record *out);
static i32   sys_svc_list(struct appf_service_record *out, u32 max_entries);
static i32   sys_hook_bind(u32 hook_type, const char *service_name);
static i32   sys_hook_emit(u32 hook_type, const void *data, u32 len);
static i32   sys_queue_create(const char *name, u32 depth, u32 flags, u32 frame_max);
static i32   sys_queue_push(i32 qid, const void *data, u32 len);
static i32   sys_queue_pop(i32 qid, void *out, u32 out_max);
static i32   sys_queue_len(i32 qid);
static i32   sys_stack_create(const char *name, u32 depth, u32 flags, u32 frame_max);
static i32   sys_stack_push(i32 sid, const void *data, u32 len);
static i32   sys_stack_pop(i32 sid, void *out, u32 out_max);
static i32   sys_stack_len(i32 sid);
static i32   sys_topic_create(const char *name, u32 replay_window, u32 flags, u32 event_max);
static i32   sys_topic_publish(i32 tid, const void *data, u32 len);
static i32   sys_topic_subscribe(i32 tid);
static i32   sys_topic_read(i32 sub_id, void *out, u32 out_max);
static i32   sys_pipe_create(const char *path, u32 type, u32 depth, u32 flags, u32 frame_max);
static i32   sys_pipe_open(const char *path, u32 type);
static i32   sys_pipe_close(i32 pipe_id);
static i32   sys_pipe_read(i32 pipe_id, void *buf, u32 len);
static i32   sys_pipe_write(i32 pipe_id, const void *buf, u32 len);
static i32   sys_pipe_send(i32 pipe_id, const void *msg, u32 len);
static i32   sys_pipe_recv(i32 pipe_id, void *msg, u32 len);
static i32   sys_pipe_stat(i32 pipe_id, struct pipe_stat *out);
static i32   sys_ipc_fifo_create(const char *name, u32 peer_principal, u32 owner_acl,
                                 u32 peer_acl, u32 depth, u32 msg_max);
static i32   sys_ipc_fifo_open(const char *name, u32 want_acl);
static i32   sys_ipc_fifo_send(i32 channel_id, const void *data, u32 len);
static i32   sys_ipc_fifo_recv(i32 channel_id, void *out, u32 out_max);
static i32   sys_ipc_shm_create(const char *name, u32 peer_principal, u32 owner_acl,
                                u32 peer_acl, u32 size);
static i32   sys_ipc_shm_open(const char *name, u32 want_acl);
static i32   sys_ipc_shm_map(i32 region_id, u32 flags, void **addr_out, u32 *size_out);
static i32   sys_ipc_shm_unmap(i32 map_handle);
static i32   sys_tensor_alloc(void *t, u32 rows, u32 cols, u32 elem_size);
static void  sys_tensor_free(void *t);
static void  sys_tensor_upload(void *t, const void *data);
static void  sys_tensor_download(const void *t, void *data);
static i32   sys_tensor_matmul(void *c, const void *a, const void *b);
static i32   sys_tensor_relu(void *b, const void *a);
static i32   sys_tensor_softmax(void *b, const void *a);
static i32   sys_tensor_add(void *c, const void *a, const void *b);
static i32   sys_tensor_dot(void *result, const void *a, const void *b);
static i32   sys_tensor_mul(void *c, const void *a, const void *b);
static i32   sys_tensor_scale(void *b, const void *a, float scalar);
static i32   sys_tensor_bind_kernel_blob(u32 kernel_id, const void *uniform_data, u32 uniform_bytes,
                                         const u64 *shader_code, u32 shader_insts);
static void  proc_tick_hook(u32 core, u64 tick);
static void  proc_preempt_trampoline(void);

#define APPF_EVENT_RING_SIZE    64U
#define APPF_LOG_RING_SIZE      64U
#define APPF_SERVICE_MAX        32U
#define APPF_HOOK_BIND_MAX      32U

struct appf_ring_event {
    struct appf_event_record recs[APPF_EVENT_RING_SIZE];
    u32 head;
    u32 tail;
    u32 seq;
};

struct appf_ring_log {
    struct appf_log_record recs[APPF_LOG_RING_SIZE];
    u32 head;
    u32 tail;
    u32 seq;
};

struct appf_service_entry {
    bool used;
    struct appf_service_record rec;
    bool capsule_enabled;
    u32 capsule_manifest_hash;
};

struct appf_hook_binding {
    bool used;
    u32 hook_type;
    char service_name[APPF_SERVICE_NAME_MAX + 1];
    u32 owner_principal;
    bool capsule_enabled;
    u32 capsule_manifest_hash;
};

struct ipc_ns_entry {
    bool used;
    u32 owner_principal;
    bool capsule_enabled;
    u32 capsule_manifest_hash;
};

static struct appf_ring_event appf_events[3];
static struct appf_ring_log appf_logs[3];
static struct appf_service_entry appf_services[3][APPF_SERVICE_MAX];
static struct appf_hook_binding appf_hooks[3][APPF_HOOK_BIND_MAX];
static struct ipc_ns_entry ipc_queue_ns[3][IPC_QUEUE_MAX_OBJECTS];
static struct ipc_ns_entry ipc_topic_ns[3][IPC_TOPIC_MAX];
static struct {
    bool capsule_enabled;
    u32 capsule_manifest_hash;
} appf_event_ns[3][APPF_EVENT_RING_SIZE], appf_log_ns[3][APPF_LOG_RING_SIZE];

static inline u32 appf_core_slot(void)
{
    return core_id() - CORE_USERM;
}

static bool appf_name_eq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        a++;
        b++;
    }
    return *a == 0 && *b == 0;
}

static void appf_name_copy(char *dst, const char *src, u32 max)
{
    u32 i = 0;
    if (!dst || !src || max == 0) return;
    while (src[i] && i + 1 < max) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = 0;
}

static bool capsule_namespace_visible(const struct process *viewer, bool owner_capsule, u32 owner_hash)
{
    if (!viewer) return false;
    if (!viewer->capsule_enabled) return true;
    return owner_capsule && owner_hash == viewer->capsule_manifest_hash;
}

static struct process *current_process(void)
{
    return &procs[current_proc];
}

static bool ipc_ns_handle_visible(const struct ipc_ns_entry *e)
{
    struct process *me = current_process();
    if (!e || !e->used) return false;
    return capsule_namespace_visible(me, e->capsule_enabled, e->capsule_manifest_hash);
}

static void ipc_ns_bind_handle(struct ipc_ns_entry *e)
{
    struct process *me = current_process();
    if (!e || !me) return;
    e->used = true;
    e->owner_principal = principal_current();
    e->capsule_enabled = me->capsule_enabled;
    e->capsule_manifest_hash = me->capsule_manifest_hash;
}

static bool topic_handle_visible(i32 tid)
{
    if (tid < 0 || tid >= IPC_TOPIC_MAX) return false;
    u32 cs = appf_core_slot();
    return ipc_ns_handle_visible(&ipc_topic_ns[cs][(u32)tid]);
}

static i32 topic_handle_from_sub(i32 sub_id)
{
    if (sub_id < 0) return -1;
    u32 raw = (u32)sub_id;
    u32 topic = (raw >> 8) & 0xFFU;
    u32 sub1 = raw & 0xFFU;
    if (sub1 == 0 || topic >= IPC_TOPIC_MAX) return -1;
    return (i32)topic;
}

static struct kernel_api kernel_api_tab = {
    /* Process control */
    .yield           = sys_yield,
    .exit            = sys_exit,
    .getpid          = sys_getpid,
    /* Console I/O */
    .print           = sys_print,
    .putc            = sys_putc,
    .getc            = sys_getc,
    .try_getc        = sys_try_getc,
    /* Timer */
    .ticks           = sys_ticks,
    .sleep_ms        = sys_sleep_ms,
    .sleep_us        = sys_sleep_us,
    .runtime_ms      = sys_runtime_ms,
    .monotonic_ms    = sys_monotonic_ms,
    .utc_ms          = sys_utc_ms,
    .set_utc_ms      = sys_set_utc_ms,
    .rtc_ms          = sys_rtc_ms,
    .set_tz_offset_min = sys_set_tz_offset_min,
    .get_tz_offset_min = sys_get_tz_offset_min,
    .list_tz_offsets = sys_list_tz_offsets,
    /* Filesystem */
    .open            = sys_open,
    .creat           = sys_creat,
    .read            = sys_read,
    .write           = sys_write,
    .pread           = sys_pread,
    .pwrite          = sys_pwrite,
    .close           = sys_close,
    .stat            = sys_stat,
    .mkdir           = sys_mkdir,
    .unlink          = sys_unlink,
    .readdir         = sys_readdir,
    .page_open       = sys_page_open,
    .page_read       = sys_page_read,
    .page_write      = sys_page_write,
    .page_flush      = sys_page_flush,
    .page_stat       = sys_page_stat,
    .page_close      = sys_page_close,
    /* Framebuffer */
    .fb_putc         = sys_fb_putc,
    .fb_print        = sys_fb_print,
    .fb_color        = sys_fb_color,
    .fb_clear        = sys_fb_clear,
    .fb_pixel        = sys_fb_pixel,
    /* Networking */
    .socket          = sys_socket,
    .bind            = sys_bind,
    .connect         = sys_connect,
    .listen          = sys_listen,
    .accept          = sys_accept,
    .send            = sys_send,
    .recv            = sys_recv,
    .sendto          = sys_sendto,
    .recvfrom        = sys_recvfrom,
    .sock_close      = sys_sock_close,
    /* DNS */
    .resolve         = sys_resolve,
    /* Identity */
    .whoami          = sys_whoami,
    .auth            = sys_auth,
    /* Memory */
    .sbrk            = sys_sbrk,
    .memset          = sys_memset,
    .memcpy          = sys_memcpy,
    .strlen          = sys_strlen,
    /* Process management */
    .spawn           = sys_spawn,
    .wait            = sys_wait,
    .nprocs          = sys_nprocs,
    /* Semaphores */
    .sem_create      = sys_sem_create,
    .sem_wait        = sys_sem_wait,
    .sem_post        = sys_sem_post,
    .lock_create     = sys_lock_create,
    .lock_acquire    = sys_lock_acquire,
    .lock_release    = sys_lock_release,
    .kv_put          = sys_kv_put,
    .kv_get          = sys_kv_get,
    .kv_del          = sys_kv_del,
    .kv_list         = sys_kv_list,
    .event_emit      = sys_event_emit,
    .event_next      = sys_event_next,
    .log_write       = sys_log_write,
    .log_next        = sys_log_next,
    .svc_register    = sys_svc_register,
    .svc_resolve     = sys_svc_resolve,
    .svc_list        = sys_svc_list,
    .hook_bind       = sys_hook_bind,
    .hook_emit       = sys_hook_emit,
    /* In-memory IPC */
    .queue_create    = sys_queue_create,
    .queue_push      = sys_queue_push,
    .queue_pop       = sys_queue_pop,
    .queue_len       = sys_queue_len,
    .stack_create    = sys_stack_create,
    .stack_push      = sys_stack_push,
    .stack_pop       = sys_stack_pop,
    .stack_len       = sys_stack_len,
    .topic_create    = sys_topic_create,
    .topic_publish   = sys_topic_publish,
    .topic_subscribe = sys_topic_subscribe,
    .topic_read      = sys_topic_read,
    /* Unified IPC pipes */
    .pipe_create     = sys_pipe_create,
    .pipe_open       = sys_pipe_open,
    .pipe_close      = sys_pipe_close,
    .pipe_read       = sys_pipe_read,
    .pipe_write      = sys_pipe_write,
    .pipe_send       = sys_pipe_send,
    .pipe_recv       = sys_pipe_recv,
    .pipe_stat       = sys_pipe_stat,
    /* Kernel-enforced IPC */
    .ipc_fifo_create = sys_ipc_fifo_create,
    .ipc_fifo_open   = sys_ipc_fifo_open,
    .ipc_fifo_send   = sys_ipc_fifo_send,
    .ipc_fifo_recv   = sys_ipc_fifo_recv,
    .ipc_shm_create  = sys_ipc_shm_create,
    .ipc_shm_open    = sys_ipc_shm_open,
    .ipc_shm_map     = sys_ipc_shm_map,
    .ipc_shm_unmap   = sys_ipc_shm_unmap,
    /* Tensor */
    .tensor_alloc    = sys_tensor_alloc,
    .tensor_free     = sys_tensor_free,
    .tensor_upload   = sys_tensor_upload,
    .tensor_download = sys_tensor_download,
    .tensor_matmul   = sys_tensor_matmul,
    .tensor_relu     = sys_tensor_relu,
    .tensor_softmax  = sys_tensor_softmax,
    .tensor_add      = sys_tensor_add,
    .tensor_dot      = sys_tensor_dot,
    .tensor_mul      = sys_tensor_mul,
    .tensor_scale    = sys_tensor_scale,
    .tensor_bind_kernel_blob = sys_tensor_bind_kernel_blob,
};

static u8 *core_ram_base(void)
{
    return (u8 *)(usize)core_ram_bases[core_id() & 3];
}

static u8 *slot_base(u32 slot)
{
    return core_ram_base() + PROC_SLOT_OFFSET + (u64)slot * PROC_SLOT_SIZE;
}

static i32 find_empty_slot(void)
{
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        if (procs[i].state == PROC_EMPTY)
            return (i32)i;
    }
    return -1;
}

/* Trampoline: x19=&kernel_api_tab, x20=entry. First schedule lands here via LR. */
static NORETURN void proc_trampoline(void)
{
    struct kernel_api *tab;
    u64 entry;
    __asm__ volatile("mov %0, x19" : "=r"(tab));
    __asm__ volatile("mov %0, x20" : "=r"(entry));

    ((void (*)(struct kernel_api *))entry)(tab);
    proc_exit(0);  /* if process returns */
    __builtin_unreachable();
}

static void proc_preempt_trampoline(void)
{
    proc_yield();
}

void proc_init(void)
{
    u32 uc = on_user_core() ? user_core_slot() : 0;
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        procs[i].state = PROC_EMPTY;
        procs[i].pid = 0;
    }
    for (u32 i = 0; i < MAX_PAGED_IO_HANDLES; i++)
        paged_io_tab[i].used = false;
    current_proc = 0;
    rr_cursor = 0;
    next_pid = (core_id() << 16) | 1;  /* encode core in upper bits */
    proc_el1_integrity_baseline = proc_el1_integrity_hash_now();
    proc_el1_integrity_next_check_tick = timer_ticks() + 512ULL;
    simd_zero(&proc_sec_stats, sizeof(proc_sec_stats));
    initialized = true;
    if (on_user_core()) {
        preempt_enabled[uc] = false;
        preempt_armed[uc] = false;
        preempt_pending[uc] = false;
        preempt_quantum_ticks[uc] = 1;
        preempt_count_core[uc] = 0;
        launch_req[uc].pending = 0;
        launch_req[uc].result_pid = -1;
        launch_req[uc].principal_id = PRINCIPAL_ROOT;
        launch_req[uc].has_principal = 0;
        launch_req[uc].priority_class = PROC_PRIO_NORMAL;
        launch_req[uc].has_priority = 0;
        launch_req[uc].migrate_keep_pid = 0;
        launch_req[uc].migrate_pid = 0;
        launch_req[uc].migrate_parent_pid = 0;
        launch_req[uc].migrate_runtime_ticks = 0;
        launch_req[uc].migrate_preemptions = 0;
        launch_req[uc].migrate_quota_mem_kib = 0;
        launch_req[uc].migrate_quota_cpu_ms = 0;
        launch_req[uc].migrate_quota_ipc_objs = 0;
        launch_req[uc].migrate_quota_fs_write_kib = 0;
        launch_req[uc].migrate_usage_ipc_objs = 0;
        launch_req[uc].migrate_usage_fs_write_bytes = 0;
        launch_req[uc].migrate_heap_used = 0;
        launch_req[uc].migrate_exec_image_size = 0;
        launch_req[uc].migrate_exec_hash_baseline = 0;
        launch_req[uc].migrate_exec_hash_last = 0;
        launch_req[uc].migrate_exec_hash_next_check_tick = 0;
        launch_req[uc].migrate_exec_hash_check_nonce = 0;
        launch_req[uc].has_migrate_state = 0;
        launch_req[uc].path[0] = 0;
        simd_zero(&appf_events[uc], sizeof(appf_events[uc]));
        simd_zero(&appf_logs[uc], sizeof(appf_logs[uc]));
        simd_zero(&appf_services[uc], sizeof(appf_services[uc]));
        simd_zero(&appf_hooks[uc], sizeof(appf_hooks[uc]));
        simd_zero(&ipc_queue_ns[uc], sizeof(ipc_queue_ns[uc]));
        simd_zero(&ipc_topic_ns[uc], sizeof(ipc_topic_ns[uc]));
        simd_zero(&appf_event_ns[uc], sizeof(appf_event_ns[uc]));
        simd_zero(&appf_log_ns[uc], sizeof(appf_log_ns[uc]));
    }
    ksem_init_core();
    workq_init_core();
    mmu_switch_to_kernel();
}

static i32 proc_exec_with_policy(const char *path, u32 priority_class, u32 affinity_core)
{
    if (!initialized)
        return -1;
    if (!proc_prio_valid(priority_class))
        return -1;
    if (affinity_core != core_id())
        return -1;

    u32 pid = principal_current();
    if (!principal_has_cap(pid, PRINCIPAL_EXEC)) {
        uart_puts("[proc] exec denied: no EXEC cap\n");
        return -1;
    }

    i32 slot = find_empty_slot();
    if (slot < 0) {
        uart_puts("[proc] no free slot\n");
        return -1;
    }

    u64 inode = walfs_find(path);
    if (inode == 0) {
        uart_puts("[proc] file not found: ");
        uart_puts(path);
        uart_putc('\n');
        return -1;
    }

    struct walfs_inode info;
    if (!walfs_stat(inode, &info)) {
        uart_puts("[proc] stat failed\n");
        return -1;
    }

    if (info.size == 0 || info.size > PROC_SLOT_SIZE - 64) {
        uart_puts("[proc] invalid binary size\n");
        return -1;
    }

    u8 *base = slot_base((u32)slot);
    dma_zero(5, base, PROC_SLOT_SIZE); /* DMA ch5 (SPARE) */
    u32 loaded = walfs_read(inode, 0, base, (u32)info.size);
    if (loaded != (u32)info.size) {
        uart_puts("[proc] load incomplete\n");
        return -1;
    }
    u32 exec_hash = hw_crc32c(base, loaded);

    struct process *p = &procs[slot];
    p->pid = next_pid++;
    p->parent_pid = 0;
    p->state = PROC_READY;
    p->principal_id = principal_current();
    p->affinity_core = affinity_core;
    p->priority_class = priority_class;
    p->quantum_ticks = proc_quantum_for_prio(priority_class);
    p->base = base;
    p->mem_size = PROC_SLOT_SIZE;
    p->ticks = timer_ticks();
    p->runtime_ticks = 0;
    p->exit_code = 0;
    p->preemptions = 0;
    p->ipc_shm_map_refs = 0;
    p->capsule_id = PROC_CAPSULE_ID_NONE;
    p->exec_image_size = loaded;
    p->exec_hash_baseline = exec_hash;
    p->exec_hash_last = exec_hash;
    p->exec_hash_check_nonce = 1;
    p->exec_hash_next_check_tick = proc_integrity_next_tick(p->pid, p->exec_hash_check_nonce);
    p->image_path[0] = 0;
    for (u32 pi = 0; pi + 1 < sizeof(p->image_path) && path[pi]; pi++) {
        p->image_path[pi] = path[pi];
        p->image_path[pi + 1] = 0;
    }
    if (!capsule_manifest_load(p, path)) {
        uart_puts("[proc] invalid capsule manifest\n");
        p->state = PROC_EMPTY;
        p->pid = 0;
        return -1;
    }
    if (p->capsule_enabled) {
        u32 cid = PROC_CAPSULE_ID_NONE;
        if (el2_capsule_bind_slot(p->principal_id, p->capsule_manifest_hash,
                                  (u64)(usize)base, PROC_SLOT_SIZE, &cid) == 0) {
            p->capsule_id = cid;
        } else {
            uart_puts("[proc] capsule bind failed\n");
            p->state = PROC_EMPTY;
            p->pid = 0;
            return -1;
        }
    }

    heap_top[(u32)slot] = ((u64)(usize)base + loaded + 15) & ~15UL;

    simd_zero(&p->ctx, sizeof(p->ctx));
    p->ctx.x19_x30[0] = (u64)(usize)&kernel_api_tab;  /* x19 */
    p->ctx.x19_x30[1] = (u64)(usize)base;           /* x20 */
    p->ctx.x19_x30[11] = (u64)(usize)proc_trampoline; /* x30 = LR */
    p->ctx.sp = (u64)(usize)(base + PROC_SLOT_SIZE - 16);

    if (!mmu_user_table_build(core_id(), (u32)slot, (u64)(usize)base, PROC_SLOT_SIZE)) {
        p->state = PROC_EMPTY;
        p->pid = 0;
        return -1;
    }

    uart_puts("[proc] loaded pid=");
    uart_hex(p->pid);
    uart_puts(" path=");
    uart_puts(path);
    uart_putc('\n');

    return (i32)p->pid;
}

i32 proc_exec(const char *path)
{
    return proc_exec_with_policy(path, PROC_PRIO_NORMAL, core_id());
}

void proc_yield(void)
{
    bool user = on_user_core();
    u32 uc = user ? user_core_slot() : 0;
    struct process *p = &procs[current_proc];
    if (user) {
        preempt_armed[uc] = false;
        preempt_pending[uc] = false;
    }
    proc_account_runtime(p);
    if (p->state == PROC_RUNNING)
        p->state = PROC_READY;
    ctx_switch(&p->ctx, &scheduler_ctx);
    if (user && preempt_enabled[uc] && p->state == PROC_RUNNING)
        preempt_armed[uc] = true;
}

NORETURN void proc_exit(u32 code)
{
    if (on_user_core()) {
        u32 uc = user_core_slot();
        preempt_armed[uc] = false;
        preempt_pending[uc] = false;
    }
    struct process *p = &procs[current_proc];
    proc_account_runtime(p);
    p->state = PROC_DEAD;
    p->exit_code = code;

    uart_puts("[proc] pid=");
    uart_hex(p->pid);
    uart_puts(" exit=");
    uart_hex(code);
    uart_putc('\n');
    ctx_switch(&p->ctx, &scheduler_ctx);
    __builtin_unreachable();
}

void proc_schedule(void)
{
    bool user = on_user_core();
    u32 uc = user ? user_core_slot() : 0;
    if (!initialized)
        proc_init();

    uart_puts("[proc] scheduler running on core ");
    uart_hex(core_id());
    uart_putc('\n');

    for (;;) {
        workq_drain(8);
        proc_handle_launch_request();
        bool found = false;
        for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
            if (procs[i].state == PROC_DEAD) {
                if (procs[i].pid != 0) {
                    u64 out = 0;
                    (void)el2_hvc_call(EL2_HVC_PORT_UNBIND_ALL, procs[i].pid, 0, 0, 0, &out);
                }
                procs[i].state = PROC_EMPTY;
                procs[i].pid = 0;
            }
        }

        bool has_non_lazy_ready = false;
        for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
            if (procs[i].state == PROC_READY && procs[i].priority_class != PROC_PRIO_LAZY) {
                has_non_lazy_ready = true;
                break;
            }
        }

        u32 chosen = 0xFFFFFFFFU;
        u32 best_prio = 0;
        for (u32 step = 0; step < MAX_PROCS_PER_CORE; step++) {
            u32 i = (rr_cursor + 1 + step) % MAX_PROCS_PER_CORE;
            if (procs[i].state != PROC_READY)
                continue;
            if (has_non_lazy_ready && procs[i].priority_class == PROC_PRIO_LAZY)
                continue;
            if (chosen == 0xFFFFFFFFU || procs[i].priority_class > best_prio) {
                chosen = i;
                best_prio = procs[i].priority_class;
            }
        }

        if (chosen != 0xFFFFFFFFU) {
            found = true;
            if (!proc_integrity_maybe_check(chosen))
                continue;
            rr_cursor = chosen;
            current_proc = chosen;
            procs[chosen].state = PROC_RUNNING;
            procs[chosen].ticks = timer_ticks();
            principal_set_current(procs[chosen].principal_id);
            if (procs[chosen].capsule_id != PROC_CAPSULE_ID_NONE) {
                if (el2_stage2_activate(procs[chosen].capsule_id) != 0) {
                    procs[chosen].state = PROC_DEAD;
                    procs[chosen].exit_code = 0xFFFF0003U;
                    principal_set_current(PRINCIPAL_ROOT);
                    continue;
                }
            } else {
                if (el2_stage2_activate(PROC_CAPSULE_ID_NONE) != 0) {
                    procs[chosen].state = PROC_DEAD;
                    procs[chosen].exit_code = 0xFFFF0004U;
                    principal_set_current(PRINCIPAL_ROOT);
                    continue;
                }
            }
            if (!mmu_switch_to_user(core_id(), chosen)) {
                procs[chosen].state = PROC_DEAD;
                procs[chosen].exit_code = 0xFFFF0002U;
                mmu_switch_to_kernel();
                principal_set_current(PRINCIPAL_ROOT);
                (void)el2_stage2_activate(PROC_CAPSULE_ID_NONE);
            } else {
                if (user && preempt_enabled[uc]) {
                    preempt_pending[uc] = false;
                    preempt_armed[uc] = true;
                }
                ctx_switch(&scheduler_ctx, &procs[chosen].ctx);
                if (user) {
                    preempt_armed[uc] = false;
                    preempt_pending[uc] = false;
                }
                mmu_switch_to_kernel();
                principal_set_current(PRINCIPAL_ROOT);
                (void)el2_stage2_activate(PROC_CAPSULE_ID_NONE);
            }
        }

        if (!found) {
            workq_drain(8);
            wfe();
        }
    }
}

bool proc_handle_fault(u64 esr, u64 elr, u64 far)
{
    if (!initialized)
        return false;
    if ((core_id() != CORE_USERM && core_id() != CORE_USER0 && core_id() != CORE_USER1))
        return false;
    u32 uc = user_core_slot();
    struct process *p = &procs[current_proc];
    if (p->state != PROC_RUNNING)
        return false;

    preempt_armed[uc] = false;
    preempt_pending[uc] = false;
    proc_account_runtime(p);
    p->state = PROC_DEAD;
    p->exit_code = 0xFFFF0001U;
    mmu_switch_to_kernel();
    principal_set_current(PRINCIPAL_ROOT);
    (void)el2_stage2_activate(PROC_CAPSULE_ID_NONE);

    uart_puts("[proc] fault pid=");
    uart_hex(p->pid);
    uart_puts(" esr=");
    uart_hex(esr);
    uart_puts(" elr=");
    uart_hex(elr);
    uart_puts(" far=");
    uart_hex(far);
    uart_putc('\n');

    ctx_switch(&p->ctx, &scheduler_ctx);
    return true;
}

u32 proc_count(void)
{
    u32 n = 0;
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        if (procs[i].state == PROC_READY || procs[i].state == PROC_RUNNING ||
            procs[i].state == PROC_BLOCKED)
            n++;
    }
    return n;
}

void proc_preempt_init(u32 timer_hz, u32 quantum_ms)
{
    if (!on_user_core())
        return;

    if (timer_hz == 0)
        timer_hz = 1;
    if (quantum_ms == 0)
        quantum_ms = 1;

    u32 uc = user_core_slot();
    u64 q = ((u64)timer_hz * (u64)quantum_ms + 999UL) / 1000UL;
    if (q == 0)
        q = 1;

    preempt_quantum_ticks[uc] = q;
    preempt_pending[uc] = false;
    preempt_armed[uc] = false;
    preempt_enabled[uc] = true;
    timer_set_tick_hook(proc_tick_hook);

    uart_puts("[proc] preempt core=");
    uart_hex(core_id());
    uart_puts(" quantum_ticks=");
    uart_hex(q);
    uart_putc('\n');
}

static void proc_tick_hook(u32 core, u64 tick)
{
    if (core != CORE_USERM && core != CORE_USER0 && core != CORE_USER1)
        return;

    u32 uc = core - CORE_USERM;
    if (!preempt_enabled[uc] || !preempt_armed[uc] || preempt_pending[uc])
        return;

    struct process *p = &procs[current_proc];
    if (p->state != PROC_RUNNING)
        return;

    u64 elapsed = tick - p->ticks;
    if (elapsed >= p->quantum_ticks)
        preempt_pending[uc] = true;
}

void proc_irq_maybe_preempt(struct irq_frame *frame)
{
    if (!frame || !on_user_core())
        return;

    u32 uc = user_core_slot();
    if (!preempt_enabled[uc] || !preempt_armed[uc] || !preempt_pending[uc])
        return;

    struct process *p = &procs[current_proc];
    if (p->state != PROC_RUNNING) {
        preempt_pending[uc] = false;
        return;
    }

    preempt_pending[uc] = false;
    proc_account_runtime(p);
    p->state = PROC_READY;
    p->preemptions++;
    preempt_count_core[uc]++;
    frame->x[30] = frame->elr;
    frame->elr = (u64)(usize)proc_preempt_trampoline;
}

u64 proc_preemptions(void)
{
    if (!on_user_core())
        return 0;
    return preempt_count_core[user_core_slot()];
}

u32 proc_snapshot(struct proc_ui_entry *out, u32 max_entries)
{
    if (!out || max_entries == 0)
        return 0;

    struct process snap[MAX_PROCS_PER_CORE];
    u32 n = 0;
    u64 total_runtime = 0;
    u64 now = timer_ticks();

    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        struct process p = procs[i];
        if (p.state != PROC_READY && p.state != PROC_RUNNING && p.state != PROC_BLOCKED)
            continue;
        if (p.state == PROC_RUNNING && now >= p.ticks)
            p.runtime_ticks += (now - p.ticks);
        snap[n++] = p;
        total_runtime += p.runtime_ticks;
        if (n == MAX_PROCS_PER_CORE)
            break;
    }

    if (total_runtime == 0)
        total_runtime = 1;

    u32 out_n = (n < max_entries) ? n : max_entries;
    for (u32 i = 0; i < out_n; i++) {
        out[i].pid = snap[i].pid;
        out[i].principal_id = snap[i].principal_id;
        out[i].state = snap[i].state;
        out[i].affinity_core = snap[i].affinity_core;
        out[i].priority_class = snap[i].priority_class;
        out[i].mem_kib = snap[i].mem_size >> 10;
        out[i].cpu_percent = (u32)((snap[i].runtime_ticks * 100ULL) / total_runtime);
        out[i].preemptions = snap[i].preemptions;
        out[i].runtime_ticks = snap[i].runtime_ticks;
    }
    return out_n;
}

u32 proc_capsule_snapshot(struct proc_capsule_ui_entry *out, u32 max_entries)
{
    if (!out || max_entries == 0)
        return 0;
    u32 n = 0;
    for (u32 i = 0; i < MAX_PROCS_PER_CORE && n < max_entries; i++) {
        struct process *p = &procs[i];
        if (p->state != PROC_READY && p->state != PROC_RUNNING && p->state != PROC_BLOCKED)
            continue;
        out[n].pid = p->pid;
        out[n].principal_id = p->principal_id;
        out[n].state = p->state;
        out[n].affinity_core = p->affinity_core;
        out[n].capsule_id = p->capsule_enabled ? p->capsule_id : PROC_CAPSULE_ID_NONE;
        out[n].capsule_hash = p->capsule_manifest_hash;
        for (u32 k = 0; k < sizeof(out[n].group); k++) out[n].group[k] = p->capsule_group[k];
        for (u32 k = 0; k < sizeof(out[n].vfs_root); k++) out[n].vfs_root[k] = p->capsule_vfs_root[k];
        n++;
    }
    return n;
}

void proc_security_stats_snapshot(struct proc_security_stats *out)
{
    if (!out) return;
    simd_memcpy(out, &proc_sec_stats, sizeof(*out));
}

bool proc_kill_pid(u32 pid, u32 code)
{
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        if (procs[i].pid != pid)
            continue;
        if (procs[i].state == PROC_READY || procs[i].state == PROC_BLOCKED ||
            procs[i].state == PROC_RUNNING) {
            if (procs[i].state == PROC_RUNNING)
                proc_account_runtime(&procs[i]);
            procs[i].state = PROC_DEAD;
            procs[i].exit_code = code;
            return true;
        }
        return false;
    }
    return false;
}

i32 proc_launch_on_core(u32 target_core, const char *path)
{
    return proc_launch_on_core_as_prio(target_core, path, principal_current(), PROC_PRIO_NORMAL);
}

i32 proc_launch_on_core_as(u32 target_core, const char *path, u32 principal_id)
{
    return proc_launch_on_core_as_prio(target_core, path, principal_id, PROC_PRIO_NORMAL);
}

i32 proc_launch_on_core_as_prio(u32 target_core, const char *path, u32 principal_id, u32 priority_class)
{
    if (!path)
        return -1;
    if (target_core != CORE_USERM && target_core != CORE_USER0 && target_core != CORE_USER1)
        return -1;
    if (!proc_prio_valid(priority_class))
        return -1;

    u32 uc = target_core - CORE_USERM;
    if (launch_req[uc].pending)
        return -1;

    u32 i = 0;
    for (; i < PROC_LAUNCH_PATH_MAX - 1 && path[i]; i++)
        launch_req[uc].path[i] = path[i];
    launch_req[uc].path[i] = 0;
    if (i == 0)
        return -1;

    launch_req[uc].principal_id = principal_id;
    launch_req[uc].has_principal = 1;
    launch_req[uc].priority_class = priority_class;
    launch_req[uc].has_priority = 1;
    launch_req[uc].migrate_keep_pid = 0;
    launch_req[uc].migrate_pid = 0;
    launch_req[uc].migrate_parent_pid = 0;
    launch_req[uc].migrate_runtime_ticks = 0;
    launch_req[uc].migrate_preemptions = 0;
    launch_req[uc].migrate_quota_mem_kib = 0;
    launch_req[uc].migrate_quota_cpu_ms = 0;
    launch_req[uc].migrate_quota_ipc_objs = 0;
    launch_req[uc].migrate_quota_fs_write_kib = 0;
    launch_req[uc].migrate_usage_ipc_objs = 0;
    launch_req[uc].migrate_usage_fs_write_bytes = 0;
    launch_req[uc].migrate_heap_used = 0;
    launch_req[uc].migrate_exec_image_size = 0;
    launch_req[uc].migrate_exec_hash_baseline = 0;
    launch_req[uc].migrate_exec_hash_last = 0;
    launch_req[uc].migrate_exec_hash_next_check_tick = 0;
    launch_req[uc].migrate_exec_hash_check_nonce = 0;
    launch_req[uc].has_migrate_state = 0;
    launch_req[uc].result_pid = -1;
    dmb();
    launch_req[uc].pending = 1;
    sev();

    u64 start = timer_ticks();
    while (launch_req[uc].pending) {
        if (timer_ticks() - start > 2000)
            return -1;
        wfe();
    }
    return launch_req[uc].result_pid;
}

bool proc_set_priority(u32 pid, u32 priority_class)
{
    if (!proc_prio_valid(priority_class))
        return false;
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        if (procs[i].pid != pid) continue;
        if (procs[i].state == PROC_READY || procs[i].state == PROC_RUNNING || procs[i].state == PROC_BLOCKED) {
            procs[i].priority_class = priority_class;
            procs[i].quantum_ticks = proc_quantum_for_prio(priority_class);
            return true;
        }
        return false;
    }
    return false;
}

bool proc_set_affinity(u32 pid, u32 core)
{
    if (core != CORE_USERM && core != CORE_USER0 && core != CORE_USER1)
        return false;
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        if (procs[i].pid != pid) continue;
        if (procs[i].state == PROC_READY || procs[i].state == PROC_RUNNING || procs[i].state == PROC_BLOCKED) {
            if (procs[i].affinity_core == core)
                return true;
            if (procs[i].image_path[0] == 0)
                return false;
            u32 uc = core - CORE_USERM;
            if (launch_req[uc].pending)
                return false;
            u32 j = 0;
            for (; j < PROC_LAUNCH_PATH_MAX - 1 && procs[i].image_path[j]; j++)
                launch_req[uc].path[j] = procs[i].image_path[j];
            launch_req[uc].path[j] = 0;
            if (j == 0)
                return false;
            if (procs[i].state == PROC_RUNNING)
                proc_account_runtime(&procs[i]);
            u64 hb = heap_top[i];
            u64 lo = (u64)(usize)procs[i].base;
            u32 heap_used = 0;
            if (hb > lo) {
                u64 d = hb - lo;
                if (d > 0xFFFFFFFFULL) d = 0xFFFFFFFFULL;
                heap_used = (u32)d;
            }
            launch_req[uc].principal_id = procs[i].principal_id;
            launch_req[uc].has_principal = 1;
            launch_req[uc].priority_class = procs[i].priority_class;
            launch_req[uc].has_priority = 1;
            launch_req[uc].migrate_keep_pid = 1;
            launch_req[uc].migrate_pid = procs[i].pid;
            launch_req[uc].migrate_parent_pid = procs[i].parent_pid;
            launch_req[uc].migrate_runtime_ticks = procs[i].runtime_ticks;
            launch_req[uc].migrate_preemptions = procs[i].preemptions;
            launch_req[uc].migrate_quota_mem_kib = procs[i].quota_mem_kib;
            launch_req[uc].migrate_quota_cpu_ms = procs[i].quota_cpu_ms;
            launch_req[uc].migrate_quota_ipc_objs = procs[i].quota_ipc_objs;
            launch_req[uc].migrate_quota_fs_write_kib = procs[i].quota_fs_write_kib;
            launch_req[uc].migrate_usage_ipc_objs = procs[i].usage_ipc_objs;
            launch_req[uc].migrate_usage_fs_write_bytes = procs[i].usage_fs_write_bytes;
            launch_req[uc].migrate_heap_used = heap_used;
            launch_req[uc].migrate_exec_image_size = procs[i].exec_image_size;
            launch_req[uc].migrate_exec_hash_baseline = procs[i].exec_hash_baseline;
            launch_req[uc].migrate_exec_hash_last = procs[i].exec_hash_last;
            launch_req[uc].migrate_exec_hash_next_check_tick = procs[i].exec_hash_next_check_tick;
            launch_req[uc].migrate_exec_hash_check_nonce = procs[i].exec_hash_check_nonce;
            launch_req[uc].has_migrate_state = 1;
            launch_req[uc].result_pid = -1;
            dmb();
            launch_req[uc].pending = 1;
            sev();
            u64 start = timer_ticks();
            while (launch_req[uc].pending) {
                if (timer_ticks() - start > 2000)
                    return false;
                wfe();
            }
            i32 new_pid = launch_req[uc].result_pid;
            if (new_pid < 0)
                return false;
            procs[i].state = PROC_DEAD;
            procs[i].exit_code = 0xFFFF0005U;
            return true;
        }
        return false;
    }
    return false;
}

/* ==== Syscall implementations ==== */

/* ---- Process control ---- */

static i32 sys_yield(void)
{
    proc_yield();
    return 0;
}

static i32 sys_exit(u32 code)
{
    proc_exit(code);
    /* unreachable */
}

static u32 sys_getpid(void)
{
    return procs[current_proc].pid;
}

/* ---- Console I/O ---- */

static void sys_print(const char *msg)
{
    if (!ptr_valid(msg, 1)) return;
    uart_puts(msg);
}

static void sys_putc(char c)          { uart_putc(c); }
static i32  sys_getc(void)            { return (i32)(u8)usb_kbd_getc(); }
static i32  sys_try_getc(void)        { return usb_kbd_try_getc(); }

/* ---- Timer ---- */

static u64  sys_ticks(void)           { return timer_ticks(); }
static void sys_sleep_ms(u64 ms)      { timer_delay_ms(ms); }
static void sys_sleep_us(u64 us)      { timer_delay_us(us); }
static u64  sys_runtime_ms(void)
{
    struct process *p = &procs[current_proc];
    u64 now = timer_ticks();
    u64 rt = p->runtime_ticks;
    if (p->state == PROC_RUNNING && now >= p->ticks)
        rt += (now - p->ticks);
    return rt;
}
static u64  sys_monotonic_ms(void)    { return timer_monotonic_ms(); }
static u64  sys_utc_ms(void)          { return timer_utc_ms(); }
static i32  sys_set_utc_ms(u64 utc_ms)
{
    if (!has_cap(PRINCIPAL_ADMIN)) return -1;
    return timer_set_utc_ms(utc_ms) ? 0 : -1;
}
static u64  sys_rtc_ms(void)          { return timer_rtc_ms(); }
static i32  sys_set_tz_offset_min(i32 offset_min)
{
    if (!has_cap(PRINCIPAL_ADMIN)) return -1;
    return timer_set_tz_offset_min(offset_min) ? 0 : -1;
}
static i32  sys_get_tz_offset_min(void) { return timer_get_tz_offset_min(); }
static i32  sys_list_tz_offsets(i32 *out_offsets, u32 max_entries)
{
    if (out_offsets && max_entries > 0) {
        if (max_entries > (0xFFFFFFFFU / (u32)sizeof(i32))) return -1;
        if (!ptr_valid(out_offsets, max_entries * (u32)sizeof(i32))) return -1;
    }
    return (i32)timer_tz_list(out_offsets, max_entries);
}

/* ---- Filesystem (WALFS via FIFO to Core 1) ---- */

static bool page_size_valid(u32 page_size)
{
    if (page_size < 512 || page_size > WALFS_DATA_MAX)
        return false;
    return (page_size & (page_size - 1U)) == 0;
}

static i32 page_handle_alloc(u64 inode_id, u32 page_size, u32 flags)
{
    u32 owner_pid = procs[current_proc].pid;
    for (u32 i = 0; i < MAX_PAGED_IO_HANDLES; i++) {
        if (!paged_io_tab[i].used) {
            paged_io_tab[i].used = true;
            paged_io_tab[i].owner_pid = owner_pid;
            paged_io_tab[i].page_size = page_size;
            paged_io_tab[i].flags = flags;
            paged_io_tab[i].inode_id = inode_id;
            return (i32)i;
        }
    }
    return -1;
}

static struct paged_io_handle *page_handle_get(i32 page_id)
{
    if (page_id < 0 || page_id >= MAX_PAGED_IO_HANDLES)
        return NULL;
    struct paged_io_handle *h = &paged_io_tab[(u32)page_id];
    if (!h->used)
        return NULL;
    if (h->owner_pid != procs[current_proc].pid)
        return NULL;
    return h;
}

static i32 sys_open(const char *path, u32 flags)
{
    (void)flags;
    if (!has_disk_cap()) return -1;
    if (!ptr_valid_cstr(path, 256)) return -1;
    char vpath[256];
    struct process *me = &procs[current_proc];
    if (!capsule_resolve_fs_path(me, path, vpath, sizeof(vpath))) return -1;
    if (!capsule_allows_fs_path(me, vpath)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_FIND;
    msg.buffer = (u64)(usize)vpath;
    msg.length = pios_strlen(vpath) + 1;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return (i32)reply.param;
}

static i32 sys_read(i32 fd, void *buf, u32 len)
{
    if (!has_disk_cap()) return -1;
    if (!ptr_valid(buf, len)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_READ;
    msg.param  = (u32)fd;
    msg.buffer = (u64)(usize)buf;
    msg.length = len;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return (i32)reply.length;
}

static i32 sys_write(i32 fd, const void *buf, u32 len)
{
    if (!has_disk_cap()) return -1;
    if (!ptr_valid(buf, len)) return -1;
    if (!capsule_quota_fs_write_allow(&procs[current_proc], len)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_WRITE;
    msg.param  = (u32)fd;
    msg.buffer = (u64)(usize)buf;
    msg.length = len;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    capsule_quota_fs_write_account(&procs[current_proc], reply.length);
    return (i32)reply.length;
}

static i32 sys_close(i32 fd) { if (!has_disk_cap()) return -1; (void)fd; return 0; }

static i32 sys_stat(const char *path, void *out)
{
    if (!has_disk_cap()) return -1;
    if (!ptr_valid_cstr(path, 256)) return -1;
    char vpath[256];
    struct process *me = &procs[current_proc];
    if (!capsule_resolve_fs_path(me, path, vpath, sizeof(vpath))) return -1;
    if (!capsule_allows_fs_path(me, vpath)) return -1;
    if (!ptr_valid(out, sizeof(struct walfs_inode))) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_STAT;
    msg.buffer = (u64)(usize)vpath;
    msg.length = pios_strlen(vpath) + 1;
    msg.tag    = (u64)(usize)out;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return 0;
}

static i32 sys_mkdir(const char *path)
{
    if (!ptr_valid_cstr(path, 256) || !has_disk_cap()) return -1;
    char vpath[256];
    struct process *me = &procs[current_proc];
    if (!capsule_resolve_fs_path(me, path, vpath, sizeof(vpath))) return -1;
    if (!capsule_allows_fs_path(me, vpath)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_MKDIR;
    msg.buffer = (u64)(usize)vpath;
    msg.length = pios_strlen(vpath) + 1;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    return reply.status == 0 ? 0 : -1;
}

static i32 sys_unlink(const char *path)
{
    if (!ptr_valid_cstr(path, 256) || !has_disk_cap()) return -1;
    char vpath[256];
    struct process *me = &procs[current_proc];
    if (!capsule_resolve_fs_path(me, path, vpath, sizeof(vpath))) return -1;
    if (!capsule_allows_fs_path(me, vpath)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_DELETE;
    msg.buffer = (u64)(usize)vpath;
    msg.length = pios_strlen(vpath) + 1;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    return reply.status == 0 ? 0 : -1;
}

static i32 sys_creat(const char *path, u32 flags, u32 mode)
{
    if (!ptr_valid_cstr(path, 256) || !has_disk_cap()) return -1;
    char vpath[256];
    struct process *me = &procs[current_proc];
    if (!capsule_resolve_fs_path(me, path, vpath, sizeof(vpath))) return -1;
    if (!capsule_allows_fs_path(me, vpath)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_CREATE;
    msg.param  = 0;
    msg.buffer = (u64)(usize)vpath;
    msg.length = pios_strlen(vpath) + 1;
    msg.tag    = ((u64)flags << 32) | mode;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return (i32)reply.param;
}

static i32 sys_pread(i32 fd, void *buf, u32 len, u64 offset)
{
    if (!has_disk_cap()) return -1;
    if (!ptr_valid(buf, len)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_READ;
    msg.param  = (u32)fd;
    msg.buffer = (u64)(usize)buf;
    msg.length = len;
    msg.tag    = offset;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return (i32)reply.length;
}

static i32 sys_pwrite(i32 fd, const void *buf, u32 len, u64 offset)
{
    if (!has_disk_cap()) return -1;
    if (!ptr_valid(buf, len)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_WRITE;
    msg.param  = (u32)fd;
    msg.buffer = (u64)(usize)buf;
    msg.length = len;
    msg.tag    = offset;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return (i32)reply.length;
}

struct readdir_entry {
    u64 inode_id;
    u8 name[128];
};

static i32 sys_readdir(const char *path, void *entries, u32 max_entries)
{
    if (!has_disk_cap()) return -1;
    if (!ptr_valid_cstr(path, 256)) return -1;
    char vpath[256];
    struct process *me = &procs[current_proc];
    if (!capsule_resolve_fs_path(me, path, vpath, sizeof(vpath))) return -1;
    if (!capsule_allows_fs_path(me, vpath)) return -1;
    if (!ptr_valid(entries, max_entries * sizeof(struct readdir_entry))) return -1;

    /* Resolve path to inode */
    struct fifo_msg fmsg = {0};
    fmsg.type   = MSG_FS_FIND;
    fmsg.buffer = (u64)(usize)vpath;
    fmsg.length = pios_strlen(vpath) + 1;
    struct fifo_msg freply;
    fs_request(&fmsg, &freply);
    if (freply.status != 0) return -1;

    /* Read directory entries */
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_READDIR;
    msg.param  = freply.param;
    msg.buffer = (u64)(usize)entries;
    msg.length = max_entries * (u32)sizeof(struct readdir_entry);
    msg.tag    = max_entries;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return (i32)reply.param;
}

static i32 sys_page_open(const char *path, u32 page_size, u32 flags)
{
    (void)flags;
    if (!has_disk_cap()) return -1;
    if (!ptr_valid_cstr(path, 256)) return -1;
    char vpath[256];
    struct process *me = &procs[current_proc];
    if (!capsule_resolve_fs_path(me, path, vpath, sizeof(vpath))) return -1;
    if (!capsule_allows_fs_path(me, vpath)) return -1;
    if (!page_size_valid(page_size)) return -1;

    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_FIND;
    msg.buffer = (u64)(usize)vpath;
    msg.length = pios_strlen(vpath) + 1;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;

    u64 inode_id = reply.tag ? reply.tag : (u64)reply.param;
    if (!inode_id) return -1;
    return page_handle_alloc(inode_id, page_size, flags);
}

static i32 sys_page_read(i32 page_id, u64 page_idx, void *out_page, u32 out_len)
{
    if (!has_disk_cap()) return -1;
    struct paged_io_handle *h = page_handle_get(page_id);
    if (!h) return -1;
    if (out_len != h->page_size) return -1;
    if (!ptr_valid(out_page, out_len)) return -1;

    u64 offset = page_idx * (u64)h->page_size;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_READ;
    msg.param  = (u32)h->inode_id;
    msg.tag    = offset;
    msg.buffer = (u64)(usize)out_page;
    msg.length = h->page_size;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return (i32)reply.length;
}

static i32 sys_page_write(i32 page_id, u64 page_idx, const void *in_page, u32 in_len)
{
    if (!has_disk_cap()) return -1;
    struct paged_io_handle *h = page_handle_get(page_id);
    if (!h) return -1;
    if (in_len != h->page_size) return -1;
    if (!ptr_valid(in_page, in_len)) return -1;

    u64 offset = page_idx * (u64)h->page_size;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_WRITE;
    msg.param  = (u32)h->inode_id;
    msg.tag    = offset;
    msg.buffer = (u64)(usize)in_page;
    msg.length = h->page_size;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return (i32)reply.length;
}

static i32 sys_page_flush(i32 page_id)
{
    if (!has_disk_cap()) return -1;
    if (!page_handle_get(page_id)) return -1;

    struct fifo_msg msg = {0};
    msg.type = MSG_FS_SYNC;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    return (reply.status == 0) ? 0 : -1;
}

static i32 sys_page_stat(i32 page_id, struct paged_io_stat *out)
{
    if (!has_disk_cap()) return -1;
    struct paged_io_handle *h = page_handle_get(page_id);
    if (!h) return -1;
    if (!ptr_valid(out, sizeof(*out))) return -1;

    struct walfs_inode ino;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_STAT;
    msg.tag    = h->inode_id;
    msg.buffer = (u64)(usize)&ino;
    msg.length = sizeof(ino);
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;

    out->inode_id = h->inode_id;
    out->file_size = ino.size;
    out->page_size = h->page_size;
    out->flags = h->flags;
    return 0;
}

static i32 sys_page_close(i32 page_id)
{
    struct paged_io_handle *h = page_handle_get(page_id);
    if (!h) return -1;
    h->used = false;
    return 0;
}

/* ---- Framebuffer ---- */

static void sys_fb_putc(char c)                    { fb_putc(c); }
static void sys_fb_print(const char *s)            { if (ptr_valid(s, 1)) fb_puts(s); }
static void sys_fb_color(u32 fg, u32 bg)           { fb_set_color(fg, bg); }
static void sys_fb_clear(u32 color)                { fb_clear(color); }
static void sys_fb_pixel(u32 x, u32 y, u32 color)  { fb_pixel(x, y, color); }

/* ---- Networking ---- */

static bool el2_port_bind_claim(struct process *p, u16 port)
{
    if (!p || port == 0) return false;
    u64 out = ~0ULL;
    if (el2_hvc_call(EL2_HVC_PORT_BIND, port, p->pid, p->capsule_manifest_hash, 0, &out) != 0)
        return false;
    return out == 0;
}

static void el2_port_unbind_claim(struct process *p, u16 port)
{
    if (!p || port == 0) return;
    u64 out = 0;
    (void)el2_hvc_call(EL2_HVC_PORT_UNBIND, port, p->pid, p->capsule_manifest_hash, 0, &out);
}

static i32 sys_socket(u32 type) { if (!has_net_cap()) return -1; return sock_socket(type); }

static i32 sys_bind(i32 fd, u32 ip, u16 port)
{
    if (!has_net_cap()) return -1;
    struct process *me = &procs[current_proc];
    if (!capsule_allows_port(me, port)) { proc_sec_stats.port_policy_denies++; return -1; }
    if (!el2_port_bind_claim(me, port)) { proc_sec_stats.port_claim_denies++; return -1; }
    struct sockaddr_in addr = { .ip = ip, .port = port };
    i32 r = sock_bind(fd, &addr);
    if (r < 0) el2_port_unbind_claim(me, port);
    return r;
}

static i32 sys_connect(i32 fd, u32 ip, u16 port)
{
    if (!has_net_cap()) return -1;
    if (!capsule_allows_port(&procs[current_proc], port)) { proc_sec_stats.port_policy_denies++; return -1; }
    struct sockaddr_in addr = { .ip = ip, .port = port };
    return sock_connect(fd, &addr);
}

static i32 sys_listen(i32 fd, u32 backlog) { if (!has_net_cap()) return -1; return sock_listen(fd, backlog); }

static i32 sys_accept(i32 fd, u32 *client_ip, u16 *client_port)
{
    if (!has_net_cap()) return -1;
    if (client_ip   && !ptr_valid(client_ip, sizeof(u32))) return -1;
    if (client_port && !ptr_valid(client_port, sizeof(u16))) return -1;
    struct sockaddr_in client = {0};
    i32 r = sock_accept(fd, &client);
    if (r >= 0) {
        if (client_ip)   *client_ip   = client.ip;
        if (client_port) *client_port = client.port;
    }
    return r;
}

static i32 sys_send(i32 fd, const void *data, u32 len)
{
    if (!has_net_cap()) return -1;
    if (!ptr_valid(data, len)) return -1;
    return sock_send(fd, data, len);
}

static i32 sys_recv(i32 fd, void *buf, u32 len)
{
    if (!has_net_cap()) return -1;
    if (!ptr_valid(buf, len)) return -1;
    return sock_recv(fd, buf, len);
}

static i32 sys_sendto(i32 fd, const void *data, u32 len, u32 ip, u16 port)
{
    if (!has_net_cap()) return -1;
    if (!ptr_valid(data, len)) return -1;
    if (!capsule_allows_port(&procs[current_proc], port)) { proc_sec_stats.port_policy_denies++; return -1; }
    struct sockaddr_in dest = { .ip = ip, .port = port };
    return sock_sendto(fd, data, len, &dest);
}

static i32 sys_recvfrom(i32 fd, void *buf, u32 len, u32 *src_ip, u16 *src_port)
{
    if (!has_net_cap()) return -1;
    if (!ptr_valid(buf, len)) return -1;
    if (src_ip   && !ptr_valid(src_ip, sizeof(u32))) return -1;
    if (src_port && !ptr_valid(src_port, sizeof(u16))) return -1;
    struct sockaddr_in src = {0};
    i32 r = sock_recvfrom(fd, buf, len, &src);
    if (r >= 0) {
        if (src_ip)   *src_ip   = src.ip;
        if (src_port) *src_port = src.port;
    }
    return r;
}

static i32 sys_sock_close(i32 fd)
{
    if (!has_net_cap()) return -1;
    u16 p = 0;
    if (sock_local_port(fd, &p))
        el2_port_unbind_claim(&procs[current_proc], p);
    return sock_close(fd);
}

/* ---- DNS ---- */

static i32 sys_resolve(const char *hostname, u32 *ip_out)
{
    if (!has_net_cap()) return -1;
    if (!ptr_valid_cstr(hostname, 254)) return -1;
    if (!ptr_valid(ip_out, sizeof(u32))) return -1;
    return dns_resolve(hostname, ip_out) ? 0 : -1;
}

/* ---- Identity ---- */

static u32 sys_whoami(void) { return principal_current(); }

static i32 sys_auth(const char *user, const char *pass)
{
    if (!ptr_valid_cstr(user, 32)) return -1;
    if (!ptr_valid_cstr(pass, 128)) return -1;
    return principal_auth(user, pass, NULL) ? 0 : -1;
}

/* ---- Memory ---- */

static void *sys_sbrk(i32 increment)
{
    struct process *p = &procs[current_proc];
    u64 old = heap_top[current_proc];
    u64 new_top = old + increment;
    u64 limit = (u64)(usize)p->base + p->mem_size - 65536;
    if (p->capsule_enabled && p->quota_mem_kib > 0) {
        u64 qlim = (u64)(usize)p->base + ((u64)p->quota_mem_kib << 10);
        if (qlim < limit) limit = qlim;
    }
    if (new_top > limit || new_top < (u64)(usize)p->base)
        return (void *)(usize)-1;
    heap_top[current_proc] = new_top;
    return (void *)(usize)old;
}

/* ---- libc stubs ---- */

static void *sys_memset(void *dst, i32 c, u32 n)
{
    if (!ptr_valid(dst, n)) return dst;
    return memset(dst, c, n);
}

static void *sys_memcpy(void *dst, const void *src, u32 n)
{
    if (!ptr_valid(dst, n) || !ptr_valid(src, n)) return dst;
    return memcpy(dst, src, n);
}

static u32 sys_strlen(const char *s)
{
    if (!ptr_valid_cstr(s, 4096)) return 0;
    return pios_strlen(s);
}

/* ---- Process management ---- */

static i32 sys_spawn(const char *path)
{
    if (!ptr_valid_cstr(path, 256) || !has_cap(PRINCIPAL_EXEC)) return -1;
    struct process *me = &procs[current_proc];
    char vpath[256];
    if (!capsule_resolve_fs_path(me, path, vpath, sizeof(vpath))) return -1;
    if (me->capsule_enabled && !me->capsule_allow_spawn) return -1;
    if (!capsule_allows_fs_path(me, vpath)) return -1;
    return proc_exec(vpath);
}

static i32 sys_wait(i32 pid)
{
    if (pid <= 0) return -1;
    struct process *me = &procs[current_proc];
    if (me->capsule_enabled && !me->capsule_allow_wait) return -1;
    for (;;) {
        bool seen = false;
        for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
            if (procs[i].pid != (u32)pid)
                continue;
            if (procs[i].state == PROC_EMPTY)
                continue;
            seen = true;
            if (me->capsule_enabled &&
                (!procs[i].capsule_enabled || procs[i].capsule_manifest_hash != me->capsule_manifest_hash))
                return -1;
            if (procs[i].state == PROC_DEAD)
                return (i32)procs[i].exit_code;
        }
        if (!seen) return -1;
        proc_yield();
    }
}

static u32 sys_nprocs(void)
{
    struct process *me = &procs[current_proc];
    if (!me->capsule_enabled)
        return proc_count();
    if (!me->capsule_allow_nprocs)
        return 0;
    u32 n = 0;
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        if (!proc_is_active_state(procs[i].state))
            continue;
        if (!procs[i].capsule_enabled)
            continue;
        if (procs[i].capsule_manifest_hash != me->capsule_manifest_hash)
            continue;
        n++;
    }
    return n;
}

/* ---- Semaphores ---- */

static i32 sys_sem_create(u32 initial)
{
    /* User ABI remains create/wait/post. trywait is kernel-internal for now. */
    return ksem_create(initial, 0x7FFFFFFFU);
}

static i32 sys_sem_wait(i32 id)
{
    for (;;) {
        i32 r = ksem_trywait(id);
        if (r == KSEM_OK)
            return 0;
        if (r != KSEM_WOULD_BLOCK)
            return -1;
        proc_yield();
    }
}

static i32 sys_sem_post(i32 id)
{
    i32 r = ksem_post(id);
    return (r == KSEM_OK) ? 0 : -1;
}

static i32 sys_lock_create(void)
{
    return ksem_create(1, 1);
}

static i32 sys_lock_acquire(i32 id)
{
    return sys_sem_wait(id);
}

static i32 sys_lock_release(i32 id)
{
    return sys_sem_post(id);
}

/* ---- Local KV store (Picowal model) ---- */

static i32 sys_kv_put(u32 key, const void *data, u32 len)
{
    if (!has_disk_cap()) return -1;
    if (!data || len == 0 || len > PICOWAL_DATA_MAX) return -1;
    if (!ptr_valid(data, len)) return -1;
    u16 card = 0;
    picowal_db_unpack_key(key, &card, NULL);
    if (!capsule_allows_card(&procs[current_proc], card)) return -1;
    return picowal_db_put_key(key, data, len);
}

static i32 sys_kv_get(u32 key, void *out, u32 out_len)
{
    if (!has_disk_cap()) return -1;
    if (!out || out_len == 0 || out_len > PICOWAL_DATA_MAX) return -1;
    if (!ptr_valid(out, out_len)) return -1;
    u16 card = 0;
    picowal_db_unpack_key(key, &card, NULL);
    if (!capsule_allows_card(&procs[current_proc], card)) return -1;
    return picowal_db_get_key(key, out, out_len);
}

static i32 sys_kv_del(u32 key)
{
    if (!has_disk_cap()) return -1;
    u16 card = 0;
    picowal_db_unpack_key(key, &card, NULL);
    if (!capsule_allows_card(&procs[current_proc], card)) return -1;
    return picowal_db_delete_key(key) ? 0 : -1;
}

static i32 sys_kv_list(u16 card, u32 *out_keys, u32 max_keys)
{
    if (!has_disk_cap()) return -1;
    if (!out_keys || max_keys == 0) return -1;
    if (max_keys > (0xFFFFFFFFU / (u32)sizeof(u32))) return -1;
    if (!ptr_valid(out_keys, max_keys * (u32)sizeof(u32))) return -1;
    if (!capsule_allows_card(&procs[current_proc], card)) return -1;
    u32 n = picowal_db_list(card, out_keys, max_keys);
    for (u32 i = 0; i < n; i++) {
        u32 key = 0;
        if (!picowal_db_pack_key(card, out_keys[i], &key))
            return -1;
        out_keys[i] = key;
    }
    return (i32)n;
}

/* ---- App foundations: events, logs, service registry, hooks ---- */

static i32 sys_event_emit(u32 type, const void *data, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (len > APPF_EVENT_DATA_MAX) return -1;
    if (len > 0 && (!data || !ptr_valid(data, len))) return -1;
    u32 cs = appf_core_slot();
    struct appf_ring_event *q = &appf_events[cs];
    u32 i = q->head;
    struct appf_event_record *r = &q->recs[i];
    struct process *me = current_process();
    r->seq = ++q->seq;
    r->type = type;
    r->len = len;
    if (len > 0)
        simd_memcpy(r->data, data, len);
    appf_event_ns[cs][i].capsule_enabled = me->capsule_enabled;
    appf_event_ns[cs][i].capsule_manifest_hash = me->capsule_manifest_hash;
    q->head = (q->head + 1U) % APPF_EVENT_RING_SIZE;
    if (q->head == q->tail)
        q->tail = (q->tail + 1U) % APPF_EVENT_RING_SIZE;
    return (i32)len;
}

static i32 sys_event_next(struct appf_event_record *out)
{
    if (!has_ipc_cap()) return -1;
    if (!out || !ptr_valid(out, sizeof(*out))) return -1;
    u32 cs = appf_core_slot();
    struct appf_ring_event *q = &appf_events[cs];
    struct process *me = current_process();
    while (q->tail != q->head) {
        u32 i = q->tail;
        if (capsule_namespace_visible(me, appf_event_ns[cs][i].capsule_enabled,
                                      appf_event_ns[cs][i].capsule_manifest_hash)) {
            simd_memcpy(out, &q->recs[i], sizeof(*out));
            q->tail = (q->tail + 1U) % APPF_EVENT_RING_SIZE;
            return (i32)out->len;
        }
        q->tail = (q->tail + 1U) % APPF_EVENT_RING_SIZE;
    }
    return -1;
}

static i32 sys_log_write(u32 level, const char *msg, u32 len)
{
    if (len > APPF_LOG_MSG_MAX) return -1;
    if (len > 0 && (!msg || !ptr_valid(msg, len))) return -1;
    u32 cs = appf_core_slot();
    struct appf_ring_log *q = &appf_logs[cs];
    u32 i = q->head;
    struct appf_log_record *r = &q->recs[i];
    struct process *me = current_process();
    r->seq = ++q->seq;
    r->level = level;
    r->len = len;
    if (len > 0)
        simd_memcpy(r->msg, msg, len);
    appf_log_ns[cs][i].capsule_enabled = me->capsule_enabled;
    appf_log_ns[cs][i].capsule_manifest_hash = me->capsule_manifest_hash;
    q->head = (q->head + 1U) % APPF_LOG_RING_SIZE;
    if (q->head == q->tail)
        q->tail = (q->tail + 1U) % APPF_LOG_RING_SIZE;
    return (i32)len;
}

static i32 sys_log_next(struct appf_log_record *out)
{
    if (!out || !ptr_valid(out, sizeof(*out))) return -1;
    u32 cs = appf_core_slot();
    struct appf_ring_log *q = &appf_logs[cs];
    struct process *me = current_process();
    while (q->tail != q->head) {
        u32 i = q->tail;
        if (capsule_namespace_visible(me, appf_log_ns[cs][i].capsule_enabled,
                                      appf_log_ns[cs][i].capsule_manifest_hash)) {
            simd_memcpy(out, &q->recs[i], sizeof(*out));
            q->tail = (q->tail + 1U) % APPF_LOG_RING_SIZE;
            return (i32)out->len;
        }
        q->tail = (q->tail + 1U) % APPF_LOG_RING_SIZE;
    }
    return -1;
}

static i32 sys_svc_register(const char *name, u32 kind, u32 endpoint, u32 flags)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid_cstr(name, APPF_SERVICE_NAME_MAX + 1)) return -1;
    u32 cs = appf_core_slot();
    u32 me = principal_current();
    struct process *cp = current_process();

    for (u32 i = 0; i < APPF_SERVICE_MAX; i++) {
        struct appf_service_entry *e = &appf_services[cs][i];
        if (!e->used) continue;
        if (!appf_name_eq(e->rec.name, name)) continue;
        if (!capsule_namespace_visible(cp, e->capsule_enabled, e->capsule_manifest_hash))
            return -1;
        if (e->rec.owner_principal != me && !has_cap(PRINCIPAL_ADMIN))
            return -1;
        e->rec.kind = kind;
        e->rec.endpoint = endpoint;
        e->rec.flags = flags;
        e->rec.owner_principal = me;
        e->capsule_enabled = cp->capsule_enabled;
        e->capsule_manifest_hash = cp->capsule_manifest_hash;
        return 0;
    }
    for (u32 i = 0; i < APPF_SERVICE_MAX; i++) {
        struct appf_service_entry *e = &appf_services[cs][i];
        if (e->used) continue;
        e->used = true;
        appf_name_copy(e->rec.name, name, sizeof(e->rec.name));
        e->rec.kind = kind;
        e->rec.endpoint = endpoint;
        e->rec.flags = flags;
        e->rec.owner_principal = me;
        e->capsule_enabled = cp->capsule_enabled;
        e->capsule_manifest_hash = cp->capsule_manifest_hash;
        return 0;
    }
    return -1;
}

static i32 sys_svc_resolve(const char *name, struct appf_service_record *out)
{
    if (!ptr_valid_cstr(name, APPF_SERVICE_NAME_MAX + 1)) return -1;
    if (!out || !ptr_valid(out, sizeof(*out))) return -1;
    u32 cs = appf_core_slot();
    struct process *cp = current_process();
    for (u32 i = 0; i < APPF_SERVICE_MAX; i++) {
        struct appf_service_entry *e = &appf_services[cs][i];
        if (!e->used) continue;
        if (!appf_name_eq(e->rec.name, name)) continue;
        if (!capsule_namespace_visible(cp, e->capsule_enabled, e->capsule_manifest_hash))
            continue;
        simd_memcpy(out, &e->rec, sizeof(*out));
        return 0;
    }
    return -1;
}

static i32 sys_svc_list(struct appf_service_record *out, u32 max_entries)
{
    if (!out || max_entries == 0) return -1;
    if (max_entries > (0xFFFFFFFFU / (u32)sizeof(*out))) return -1;
    if (!ptr_valid(out, max_entries * (u32)sizeof(*out))) return -1;
    u32 cs = appf_core_slot();
    struct process *cp = current_process();
    u32 n = 0;
    for (u32 i = 0; i < APPF_SERVICE_MAX && n < max_entries; i++) {
        struct appf_service_entry *e = &appf_services[cs][i];
        if (!e->used) continue;
        if (!capsule_namespace_visible(cp, e->capsule_enabled, e->capsule_manifest_hash))
            continue;
        simd_memcpy(&out[n++], &e->rec, sizeof(*out));
    }
    return (i32)n;
}

static i32 sys_hook_bind(u32 hook_type, const char *service_name)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid_cstr(service_name, APPF_SERVICE_NAME_MAX + 1)) return -1;
    if (hook_type == 0) return -1;
    u32 cs = appf_core_slot();
    u32 me = principal_current();
    struct process *cp = current_process();

    struct appf_service_record svc;
    if (sys_svc_resolve(service_name, &svc) != 0)
        return -1;

    for (u32 i = 0; i < APPF_HOOK_BIND_MAX; i++) {
        struct appf_hook_binding *b = &appf_hooks[cs][i];
        if (!b->used) continue;
        if (b->hook_type == hook_type && appf_name_eq(b->service_name, service_name)) {
            if (!capsule_namespace_visible(cp, b->capsule_enabled, b->capsule_manifest_hash))
                return -1;
            if (b->owner_principal != me && !has_cap(PRINCIPAL_ADMIN))
                return -1;
            b->owner_principal = me;
            b->capsule_enabled = cp->capsule_enabled;
            b->capsule_manifest_hash = cp->capsule_manifest_hash;
            return 0;
        }
    }
    for (u32 i = 0; i < APPF_HOOK_BIND_MAX; i++) {
        struct appf_hook_binding *b = &appf_hooks[cs][i];
        if (b->used) continue;
        b->used = true;
        b->hook_type = hook_type;
        b->owner_principal = me;
        b->capsule_enabled = cp->capsule_enabled;
        b->capsule_manifest_hash = cp->capsule_manifest_hash;
        appf_name_copy(b->service_name, service_name, sizeof(b->service_name));
        return 0;
    }
    return -1;
}

static i32 sys_hook_emit(u32 hook_type, const void *data, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (hook_type == 0 || len > APPF_EVENT_DATA_MAX) return -1;
    if (len > 0 && (!data || !ptr_valid(data, len))) return -1;

    struct {
        u32 hook_type;
        u32 len;
        u8  data[APPF_EVENT_DATA_MAX];
    } ctx;
    ctx.hook_type = hook_type;
    ctx.len = len;
    if (len > 0)
        simd_memcpy(ctx.data, data, len);
    module_call_hooks(hook_type, &ctx);

    u32 cs = appf_core_slot();
    struct process *cp = current_process();
    for (u32 i = 0; i < APPF_HOOK_BIND_MAX; i++) {
        struct appf_hook_binding *b = &appf_hooks[cs][i];
        if (!b->used || b->hook_type != hook_type)
            continue;
        if (!capsule_namespace_visible(cp, b->capsule_enabled, b->capsule_manifest_hash))
            continue;
        struct appf_service_record svc;
        if (sys_svc_resolve(b->service_name, &svc) != 0)
            continue;
        struct {
            u32 hook_type;
            u32 endpoint;
            u32 kind;
            u32 payload_len;
            u8  payload[APPF_EVENT_DATA_MAX - 16];
        } ev;
        ev.hook_type = hook_type;
        ev.endpoint = svc.endpoint;
        ev.kind = svc.kind;
        ev.payload_len = len;
        u32 copy = len;
        if (copy > sizeof(ev.payload))
            copy = sizeof(ev.payload);
        if (copy > 0)
            simd_memcpy(ev.payload, data, copy);
        sys_event_emit(0x80000000U | hook_type, &ev, 16U + copy);
    }
    return (i32)len;
}

/* ---- In-memory IPC ---- */

static i32 sys_queue_create(const char *name, u32 depth, u32 flags, u32 frame_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid_cstr(name, IPC_NAME_MAX + 1)) return -1;
    if (!capsule_allows_ipc_name(&procs[current_proc], name)) return -1;
    if (!capsule_quota_ipc_consume(&procs[current_proc])) return -1;
    i32 h = ipc_queue_create(name, depth, flags, frame_max);
    if (h < 0 && procs[current_proc].usage_ipc_objs > 0) procs[current_proc].usage_ipc_objs--;
    if (h >= 0 && h < IPC_QUEUE_MAX_OBJECTS) {
        u32 cs = appf_core_slot();
        ipc_ns_bind_handle(&ipc_queue_ns[cs][(u32)h]);
    }
    return h;
}

static i32 sys_queue_push(i32 qid, const void *data, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(data, len)) return -1;
    if (qid < 0 || qid >= IPC_QUEUE_MAX_OBJECTS) return -1;
    if (!ipc_ns_handle_visible(&ipc_queue_ns[appf_core_slot()][(u32)qid])) return -1;
    return ipc_queue_push(qid, data, len);
}

static i32 sys_queue_pop(i32 qid, void *out, u32 out_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(out, out_max)) return -1;
    if (qid < 0 || qid >= IPC_QUEUE_MAX_OBJECTS) return -1;
    if (!ipc_ns_handle_visible(&ipc_queue_ns[appf_core_slot()][(u32)qid])) return -1;
    u32 len = 0;
    i32 r = ipc_queue_pop(qid, out, out_max, &len);
    if (r != IPC_OK) return r;
    return (i32)len;
}

static i32 sys_queue_len(i32 qid)
{
    if (!has_ipc_cap()) return -1;
    if (qid < 0 || qid >= IPC_QUEUE_MAX_OBJECTS) return -1;
    if (!ipc_ns_handle_visible(&ipc_queue_ns[appf_core_slot()][(u32)qid])) return -1;
    return ipc_queue_len(qid);
}

static i32 sys_stack_create(const char *name, u32 depth, u32 flags, u32 frame_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid_cstr(name, IPC_NAME_MAX + 1)) return -1;
    if (!capsule_allows_ipc_name(&procs[current_proc], name)) return -1;
    if (!capsule_quota_ipc_consume(&procs[current_proc])) return -1;
    i32 h = ipc_stack_create(name, depth, flags, frame_max);
    if (h < 0 && procs[current_proc].usage_ipc_objs > 0) procs[current_proc].usage_ipc_objs--;
    if (h >= 0 && h < IPC_QUEUE_MAX_OBJECTS) {
        u32 cs = appf_core_slot();
        ipc_ns_bind_handle(&ipc_queue_ns[cs][(u32)h]);
    }
    return h;
}

static i32 sys_stack_push(i32 sid, const void *data, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(data, len)) return -1;
    if (sid < 0 || sid >= IPC_QUEUE_MAX_OBJECTS) return -1;
    if (!ipc_ns_handle_visible(&ipc_queue_ns[appf_core_slot()][(u32)sid])) return -1;
    return ipc_stack_push(sid, data, len);
}

static i32 sys_stack_pop(i32 sid, void *out, u32 out_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(out, out_max)) return -1;
    if (sid < 0 || sid >= IPC_QUEUE_MAX_OBJECTS) return -1;
    if (!ipc_ns_handle_visible(&ipc_queue_ns[appf_core_slot()][(u32)sid])) return -1;
    u32 len = 0;
    i32 r = ipc_stack_pop(sid, out, out_max, &len);
    if (r != IPC_OK) return r;
    return (i32)len;
}

static i32 sys_stack_len(i32 sid)
{
    if (!has_ipc_cap()) return -1;
    if (sid < 0 || sid >= IPC_QUEUE_MAX_OBJECTS) return -1;
    if (!ipc_ns_handle_visible(&ipc_queue_ns[appf_core_slot()][(u32)sid])) return -1;
    return ipc_stack_len(sid);
}

static i32 sys_topic_create(const char *name, u32 replay_window, u32 flags, u32 event_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid_cstr(name, IPC_NAME_MAX + 1)) return -1;
    if (!capsule_allows_ipc_name(&procs[current_proc], name)) return -1;
    if (!capsule_quota_ipc_consume(&procs[current_proc])) return -1;
    i32 h = ipc_topic_create(name, replay_window, flags, event_max);
    if (h < 0 && procs[current_proc].usage_ipc_objs > 0) procs[current_proc].usage_ipc_objs--;
    if (h >= 0 && h < IPC_TOPIC_MAX) {
        u32 cs = appf_core_slot();
        ipc_ns_bind_handle(&ipc_topic_ns[cs][(u32)h]);
    }
    return h;
}

static i32 sys_topic_publish(i32 tid, const void *data, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(data, len)) return -1;
    if (!topic_handle_visible(tid)) return -1;
    return ipc_topic_publish(tid, data, len);
}

static i32 sys_topic_subscribe(i32 tid)
{
    if (!has_ipc_cap()) return -1;
    if (!topic_handle_visible(tid)) return -1;
    return ipc_topic_subscribe(tid);
}

static i32 sys_topic_read(i32 sub_id, void *out, u32 out_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(out, out_max)) return -1;
    i32 tid = topic_handle_from_sub(sub_id);
    if (!topic_handle_visible(tid)) return -1;
    u32 len = 0;
    i32 r = ipc_topic_read(sub_id, out, out_max, &len);
    if (r != IPC_OK) return r;
    return (i32)len;
}

/* ---- Unified pipes ---- */

static u32 pipe_path_domain(const char *path)
{
    if (!path || path[0] != '/') return 0;
    if (path[1] == 'i' && path[2] == 'p' && path[3] == 'c' && path[4] == '/')
        return PIPE_DOMAIN_IPC;
    if (path[1] == 'n' && path[2] == 'e' && path[3] == 't' && path[4] == '/')
        return PIPE_DOMAIN_NET;
    if (path[1] == 'f' && path[2] == 's' && path[3] == '/')
        return PIPE_DOMAIN_FS;
    if (path[1] == 'h' && path[2] == 'w' && path[3] == '/')
        return PIPE_DOMAIN_HW;
    return 0;
}

static bool pipe_path_allowed(const char *path)
{
    u32 d = pipe_path_domain(path);
    if (d == PIPE_DOMAIN_IPC) return has_ipc_cap();
    if (d == PIPE_DOMAIN_NET) return has_net_cap();
    if (d == PIPE_DOMAIN_FS) return has_disk_cap();
    if (d == PIPE_DOMAIN_HW) return has_cap(PRINCIPAL_ADMIN);
    return false;
}

static i32 sys_pipe_create(const char *path, u32 type, u32 depth, u32 flags, u32 frame_max)
{
    if (!ptr_valid_cstr(path, PIPE_PATH_MAX + 1)) return -1;
    if (!pipe_path_allowed(path)) return -1;
    if (!capsule_allows_pipe_path(&procs[current_proc], path)) return -1;
    if (!capsule_quota_ipc_consume(&procs[current_proc])) return -1;
    i32 h = pipe_create(path, type, depth, flags, frame_max);
    if (h < 0 && procs[current_proc].usage_ipc_objs > 0) procs[current_proc].usage_ipc_objs--;
    return h;
}

static i32 sys_pipe_open(const char *path, u32 type)
{
    if (!ptr_valid_cstr(path, PIPE_PATH_MAX + 1)) return -1;
    if (!pipe_path_allowed(path)) return -1;
    if (!capsule_allows_pipe_path(&procs[current_proc], path)) return -1;
    return pipe_open(path, type);
}

static i32 sys_pipe_close(i32 pipe_id)
{
    if (!has_ipc_cap()) return -1;
    return pipe_close(pipe_id);
}

static i32 sys_pipe_read(i32 pipe_id, void *buf, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(buf, len)) return -1;
    return pipe_read(pipe_id, buf, len);
}

static i32 sys_pipe_write(i32 pipe_id, const void *buf, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(buf, len)) return -1;
    return pipe_write(pipe_id, buf, len);
}

static i32 sys_pipe_send(i32 pipe_id, const void *msg, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(msg, len)) return -1;
    return pipe_send(pipe_id, msg, len);
}

static i32 sys_pipe_recv(i32 pipe_id, void *msg, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(msg, len)) return -1;
    return pipe_recv(pipe_id, msg, len);
}

static i32 sys_pipe_stat(i32 pipe_id, struct pipe_stat *out)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(out, sizeof(*out))) return -1;
    return pipe_stat(pipe_id, out);
}

/* ---- Kernel-enforced process IPC ---- */

static i32 sys_ipc_fifo_create(const char *name, u32 peer_principal, u32 owner_acl,
                               u32 peer_acl, u32 depth, u32 msg_max)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    if (!ptr_valid_cstr(name, PROC_IPC_NAME_MAX + 1)) return PROC_IPC_ERR_INVAL;
    if (!capsule_allows_ipc_name(&procs[current_proc], name)) return PROC_IPC_ERR_ACCESS;
    if (!capsule_quota_ipc_consume(&procs[current_proc])) return PROC_IPC_ERR_ACCESS;
    if (peer_principal != PROC_IPC_PEER_ANY && peer_principal >= PRINCIPAL_MAX)
        return PROC_IPC_ERR_INVAL;
    i32 h = ipc_proc_fifo_create(principal_current(), procs[current_proc].pid, name,
                                 peer_principal, owner_acl, peer_acl, depth, msg_max);
    if (h < 0 && procs[current_proc].usage_ipc_objs > 0) procs[current_proc].usage_ipc_objs--;
    return h;
}

static i32 sys_ipc_fifo_open(const char *name, u32 want_acl)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    if (!ptr_valid_cstr(name, PROC_IPC_NAME_MAX + 1)) return PROC_IPC_ERR_INVAL;
    if (!capsule_allows_ipc_name(&procs[current_proc], name)) return PROC_IPC_ERR_ACCESS;
    return ipc_proc_fifo_open(principal_current(), procs[current_proc].pid, name, want_acl);
}

static i32 sys_ipc_fifo_send(i32 channel_id, const void *data, u32 len)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    if (!ptr_valid(data, len)) return PROC_IPC_ERR_INVAL;
    return ipc_proc_fifo_send(principal_current(), channel_id, data, len);
}

static i32 sys_ipc_fifo_recv(i32 channel_id, void *out, u32 out_max)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    if (!ptr_valid(out, out_max)) return PROC_IPC_ERR_INVAL;
    u32 len = 0;
    i32 r = ipc_proc_fifo_recv(principal_current(), channel_id, out, out_max, &len);
    if (r != PROC_IPC_OK) return r;
    return (i32)len;
}

static i32 sys_ipc_shm_create(const char *name, u32 peer_principal, u32 owner_acl,
                              u32 peer_acl, u32 size)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    if (!ptr_valid_cstr(name, PROC_IPC_NAME_MAX + 1)) return PROC_IPC_ERR_INVAL;
    if (!capsule_allows_ipc_name(&procs[current_proc], name)) return PROC_IPC_ERR_ACCESS;
    if (!capsule_quota_ipc_consume(&procs[current_proc])) return PROC_IPC_ERR_ACCESS;
    if (peer_principal != PROC_IPC_PEER_ANY && peer_principal >= PRINCIPAL_MAX)
        return PROC_IPC_ERR_INVAL;
    mmu_switch_to_kernel();
    i32 r = ipc_proc_shm_create(principal_current(), procs[current_proc].pid, name,
                                peer_principal, owner_acl, peer_acl, size);
    (void)mmu_switch_to_user(core_id(), current_proc);
    if (r < 0 && procs[current_proc].usage_ipc_objs > 0) procs[current_proc].usage_ipc_objs--;
    return r;
}

static i32 sys_ipc_shm_open(const char *name, u32 want_acl)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    if (!ptr_valid_cstr(name, PROC_IPC_NAME_MAX + 1)) return PROC_IPC_ERR_INVAL;
    if (!capsule_allows_ipc_name(&procs[current_proc], name)) return PROC_IPC_ERR_ACCESS;
    return ipc_proc_shm_open(principal_current(), procs[current_proc].pid, name, want_acl);
}

static i32 sys_ipc_shm_map(i32 region_id, u32 flags, void **addr_out, u32 *size_out)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    if (!ptr_valid(addr_out, sizeof(*addr_out))) return PROC_IPC_ERR_INVAL;
    if (!ptr_valid(size_out, sizeof(*size_out))) return PROC_IPC_ERR_INVAL;
    i32 h = ipc_proc_shm_map(principal_current(), procs[current_proc].pid, region_id,
                             flags, addr_out, size_out);
    if (h < 0)
        return h;
    struct process *p = &procs[current_proc];
    if (p->ipc_shm_map_refs == 0) {
        if (!mmu_user_ipc_shm_window(core_id(), current_proc, true)) {
            (void)ipc_proc_shm_unmap(principal_current(), procs[current_proc].pid, h);
            return PROC_IPC_ERR_UNSUPPORTED;
        }
    }
    p->ipc_shm_map_refs++;
    return h;
}

static i32 sys_ipc_shm_unmap(i32 map_handle)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    i32 r = ipc_proc_shm_unmap(principal_current(), procs[current_proc].pid, map_handle);
    if (r != PROC_IPC_OK)
        return r;
    struct process *p = &procs[current_proc];
    if (p->ipc_shm_map_refs > 0) {
        p->ipc_shm_map_refs--;
        if (p->ipc_shm_map_refs == 0)
            (void)mmu_user_ipc_shm_window(core_id(), current_proc, false);
    }
    return PROC_IPC_OK;
}

/* ---- Tensor / GPU compute ---- */

static i32 sys_tensor_alloc(void *t, u32 rows, u32 cols, u32 elem_size)
{
    if (!ptr_valid(t, sizeof(tensor_t))) return -1;
    return tensor_alloc((tensor_t *)t, rows, cols, elem_size) ? 0 : -1;
}

static void sys_tensor_free(void *t)
{
    if (ptr_valid(t, sizeof(tensor_t)))
        tensor_free((tensor_t *)t);
}

static void sys_tensor_upload(void *t, const void *data)
{
    if (!ptr_valid(t, sizeof(tensor_t))) return;
    tensor_t *tp = (tensor_t *)t;
    if (!ptr_valid(data, tp->total_bytes)) return;
    tensor_upload(tp, data);
}

static void sys_tensor_download(const void *t, void *data)
{
    if (!ptr_valid(t, sizeof(tensor_t))) return;
    const tensor_t *tp = (const tensor_t *)t;
    if (!ptr_valid(data, tp->total_bytes)) return;
    tensor_download(tp, data);
}

static i32 sys_tensor_matmul(void *c, const void *a, const void *b)
{
    return tensor_matmul((tensor_t *)c, (const tensor_t *)a, (const tensor_t *)b) ? 0 : -1;
}

static i32 sys_tensor_relu(void *b, const void *a)
{
    return tensor_relu((tensor_t *)b, (const tensor_t *)a) ? 0 : -1;
}

static i32 sys_tensor_softmax(void *b, const void *a)
{
    return tensor_softmax((tensor_t *)b, (const tensor_t *)a) ? 0 : -1;
}

static i32 sys_tensor_add(void *c, const void *a, const void *b)
{
    return tensor_add((tensor_t *)c, (const tensor_t *)a, (const tensor_t *)b) ? 0 : -1;
}

static i32 sys_tensor_dot(void *result, const void *a, const void *b)
{
    if (!ptr_valid(result, sizeof(float))) return -1;
    return tensor_dot((float *)result, (const tensor_t *)a, (const tensor_t *)b) ? 0 : -1;
}

static i32 sys_tensor_mul(void *c, const void *a, const void *b)
{
    return tensor_mul((tensor_t *)c, (const tensor_t *)a, (const tensor_t *)b) ? 0 : -1;
}

static i32 sys_tensor_scale(void *b, const void *a, float scalar)
{
    return tensor_scale((tensor_t *)b, (const tensor_t *)a, scalar) ? 0 : -1;
}

static i32 sys_tensor_bind_kernel_blob(u32 kernel_id, const void *uniform_data, u32 uniform_bytes,
                                       const u64 *shader_code, u32 shader_insts)
{
    if (!has_cap(PRINCIPAL_ADMIN)) return -1;
    if (uniform_bytes == 0 || shader_insts == 0) return -1;
    if (!ptr_valid(uniform_data, uniform_bytes)) return -1;
    u64 shader_bytes = (u64)shader_insts * 8U;
    if (shader_bytes > 0xFFFFFFFFU) return -1;
    if (!ptr_valid(shader_code, (u32)shader_bytes)) return -1;
    return (i32)v3d_kernel_bind_blob((v3d_kernel_id_t)kernel_id,
                                     uniform_data, uniform_bytes,
                                     shader_code, shader_insts);
}
