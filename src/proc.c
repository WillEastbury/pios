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
#include "watchdog.h"

static struct process  procs[MAX_PROCS_PER_CORE];
static struct proc_context scheduler_ctx;
static u32 current_proc;   /* index into procs[] */
static u32 rr_cursor;
static u32 next_pid;
static bool initialized;
static u64 heap_top[MAX_PROCS_PER_CORE];
/* Sticky per-slot soft-event wake latch (semaphore semantics). Kept OUT of
 * struct process so the process-table element stride / .bss layout stays
 * byte-identical. proc_soft_event() sets it; proc_park() consumes it and
 * refuses to block while it is set, closing the lost-wakeup race that stranded
 * the port-81 httpd after its first request (504 on every subsequent one). */
static volatile u32 proc_wake_pending[MAX_PROCS_PER_CORE];
static volatile bool preempt_enabled[3];
static volatile bool preempt_armed[3];
static volatile bool preempt_pending[3];
static u64 preempt_quantum_ticks[3];
static u64 preempt_count_core[3];
static u64 sched_start_ticks_core[3];
static u64 sched_idle_ticks_core[3];
static volatile u64 sched_idle_enter_ticks_core[3];
static u64 sched_idle_count_core[3];
static u64 sched_wake_count_core[3];
static u64 soft_event_count_core[3];
static u64 soft_boost_count_core[3];
static u64 svc_call_count;
static u64 svc_bad_count;
static struct proc_security_stats proc_sec_stats;

/* ----------------------------------------------------------------------------
 * cohdiag - cross-core cache-coherency / memory-attribute diagnostic.
 *
 * Settles, on real BCM2712 hardware, whether the cross-core "doorbell data
 * visibility" problem is a coherency gap (it is not - the A76/DynamIQ DSU is
 * always coherent for Normal-WB-Inner-Shareable) or a software issue: arenas
 * mapped Normal-NC plus a kernel/user attribute mismatch. Producer runs on
 * CORE_NET (0); the consumer runs as a hook inside the target user core's
 * proc_schedule() loop, woken from WFE by SEV.
 * -------------------------------------------------------------------------- */
#define COHDIAG_WB_SCRATCH  (CORE0_RAM_BASE + 0x00C00000UL)  /* 12MB into core0's 16MB WB-IS region */

struct cohdiag_a {
    volatile u64 seq;
    volatile u64 payload[7];
    volatile u32 producer_done;
    volatile u32 consumer_started;
    volatile u32 consumer_done;
    volatile u32 checks;
    volatile u32 mismatch;
    volatile u32 tears;
    volatile u32 use_acquire;
};
static volatile struct cohdiag_a *cohdiag_target;   /* struct under test          */
static volatile u32 cohdiag_consumer_arm;           /* 0=idle else (consumer+1)   */
static volatile struct cohdiag_a cohdiag_nc_a;      /* .bss => Normal-NC alias    */
static u8 cohdiag_nc_buf[8192] ALIGNED(64);         /* .bss => Normal-NC scratch  */

#define PROC_IMAGE_VALIDATE_HEADER_MAX 4096U
static u8 proc_image_validate_scratch[PROC_IMAGE_VALIDATE_HEADER_MAX];

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
    u32 migrate_arena_high_bytes;
    u32 migrate_exec_image_size;
    u32 migrate_exec_hash_baseline;
    u32 migrate_exec_hash_last;
    u64 migrate_exec_hash_next_check_tick;
    u32 migrate_exec_hash_check_nonce;
    u32 has_migrate_state;
    char path[PROC_LAUNCH_PATH_MAX];
};
static struct proc_launch_req launch_req[3];
#define PROC_SPANS_PER_PROCESS 8U
struct proc_span_slot {
    bool used;
    u32 type;
    u32 size;
    u64 addr;
};
static struct proc_span_slot proc_spans[MAX_PROCS_PER_CORE][PROC_SPANS_PER_PROCESS];
static u64 proc_span_floor[MAX_PROCS_PER_CORE];
static inline bool on_user_core(void);
static inline u32 user_core_slot(void);
static i32 proc_exec_with_policy(const char *path, u32 priority_class, u32 affinity_core);
static void proc_drain_remote_wakes(void);
static void proc_sched_heartbeat(void);
static void proc_sched_note_ctx_enter(u32 pid);
static void proc_sched_note_ctx_exit(void);
static void proc_sched_stage(u32 s);
static void proc_account_runtime(struct process *p);
static void proc_diag_note_dispatch(void);
static void proc_diag_note_wfe(void);
static u32 proc_arena_bump_bytes(u32 slot);
static void proc_arena_update_high(struct process *p, u32 slot);
static void proc_span_reset(u32 slot);
static bool proc_entry_contract_validate(const struct process *p);

static inline u64 proc_sched_counter_ticks(void)
{
    u64 cnt;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(cnt));
    return cnt;
}

extern u8 __text_start;
extern u8 __text_end;
extern u8 _start;
extern u8 __heap_start;
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

static u32 proc_arena_bump_bytes(u32 slot)
{
    if (slot >= MAX_PROCS_PER_CORE)
        return 0;
    struct process *p = &procs[slot];
    if (p->arena_base == 0 || heap_top[slot] <= p->arena_base)
        return 0;
    u64 used = heap_top[slot] - p->arena_base;
    return used > 0xFFFFFFFFULL ? 0xFFFFFFFFU : (u32)used;
}

static u64 proc_kernel_static_bytes(void)
{
    u64 start = (u64)(usize)&_start;
    u64 heap = (u64)(usize)&__heap_start;
    return heap > start ? heap - start : 0;
}

static u64 proc_kernel_core_private_bytes(void)
{
    u64 used = 0;
    for (u32 i = 0; i < 4; i++) {
        struct core_env *e = core_env_of(i);
        if (e->id == i &&
            e->ram_base == (u8 *)(usize)core_ram_bases[i] &&
            e->ram_end == e->ram_base + CORE_PRIV_SIZE &&
            e->heap_ptr >= e->ram_base &&
            e->heap_ptr <= e->ram_end) {
            used += (u64)(usize)(e->heap_ptr - e->ram_base);
        }
    }
    return used;
}

u32 proc_entry_contract_flags(void)
{
    return PROC_ENTRY_FLAG_DIRECT_KPI |
           PROC_ENTRY_FLAG_EL0_CONTRACT |
           PROC_ENTRY_FLAG_CODE_RX_RO |
           PROC_ENTRY_FLAG_DATA_RW_XN |
           PROC_ENTRY_FLAG_STACK_16_ALIGN |
           PROC_ENTRY_FLAG_API_IN_X0 |
           PROC_ENTRY_FLAG_SVC_REQUIRED;
}

u32 proc_entry_contract_spsr(void)
{
    return PROC_ENTRY_SPSR_EL0_DAIF;
}

static bool proc_entry_contract_validate(const struct process *p)
{
    u64 base;
    u64 limit;
    if (!p || !p->base || p->mem_size != PROC_SLOT_SIZE || p->exec_image_size == 0)
        return false;
    base = (u64)(usize)p->base;
    limit = base + p->mem_size;
    if (limit <= base)
        return false;
    if ((p->entry_pc & 3ULL) != 0 || p->entry_pc < base || p->entry_pc >= base + p->exec_image_size)
        return false;
    if ((p->entry_sp & 15ULL) != 0 || p->entry_sp <= p->arena_limit || p->entry_sp >= limit)
        return false;
    if (p->arena_base <= base || p->arena_base >= p->arena_limit || p->arena_limit >= p->entry_sp)
        return false;
    if ((p->entry_flags & proc_entry_contract_flags()) != proc_entry_contract_flags())
        return false;
    return p->entry_spsr == PROC_ENTRY_SPSR_EL0_DAIF;
}

bool proc_entry_contract_selftest(void)
{
    struct process p;
    u8 *fake_slot = (u8 *)(usize)0x40000000ULL;
    simd_zero(&p, sizeof(p));
    p.base = fake_slot;
    p.mem_size = PROC_SLOT_SIZE;
    p.exec_image_size = 4096U;
    p.entry_pc = (u64)(usize)fake_slot;
    p.arena_base = ((u64)(usize)fake_slot + 4096U + L3_PAGE_SIZE - 1U) & ~(u64)(L3_PAGE_SIZE - 1U);
    p.arena_limit = (u64)(usize)fake_slot + PROC_SLOT_SIZE - 65536ULL;
    p.entry_sp = (u64)(usize)(fake_slot + PROC_SLOT_SIZE - 16U);
    p.entry_spsr = PROC_ENTRY_SPSR_EL0_DAIF;
    p.entry_flags = proc_entry_contract_flags();
    return proc_entry_contract_validate(&p);
}

#define PROC_SVC_NOP     0U
#define PROC_SVC_GETPID  1U

u64 proc_svc_calls(void)
{
    return svc_call_count;
}

u64 proc_svc_bad_calls(void)
{
    return svc_bad_count;
}

static u32 proc_current_pid_for_svc(void)
{
    if (!on_user_core() || current_proc >= MAX_PROCS_PER_CORE)
        return 0;
    if (procs[current_proc].state != PROC_RUNNING)
        return 0;
    return procs[current_proc].pid;
}

static bool proc_handle_svc_inner(struct irq_frame *frame, u64 esr, bool account)
{
    u32 imm = (u32)(esr & 0xFFFFU);
    if (!frame)
        return false;
    if (account)
        svc_call_count++;
    switch (imm) {
    case PROC_SVC_NOP:
        frame->x[0] = 0;
        return true;
    case PROC_SVC_GETPID:
        frame->x[0] = proc_current_pid_for_svc();
        return true;
    default:
        if (account)
            svc_bad_count++;
        frame->x[0] = (u64)-38; /* ENOSYS */
        return true;
    }
}

bool proc_handle_svc(struct irq_frame *frame, u64 esr)
{
    return proc_handle_svc_inner(frame, esr, true);
}

bool proc_svc_selftest(void)
{
    struct irq_frame f;
    simd_zero(&f, sizeof(f));
    if (!proc_handle_svc_inner(&f, ((u64)EC_SVC64 << ESR_EC_SHIFT) | PROC_SVC_NOP, false))
        return false;
    if (f.x[0] != 0)
        return false;
    if (!proc_handle_svc_inner(&f, ((u64)EC_SVC64 << ESR_EC_SHIFT) | 0xFFFFU, false))
        return false;
    return f.x[0] == (u64)-38;
}

static void proc_arena_update_high(struct process *p, u32 slot)
{
    if (!p || slot >= MAX_PROCS_PER_CORE)
        return;
    u32 bump = proc_arena_bump_bytes(slot);
    u64 used = (u64)bump + p->arena_span_bytes;
    if (used > 0xFFFFFFFFULL)
        used = 0xFFFFFFFFULL;
    if ((u32)used > p->arena_high_bytes)
        p->arena_high_bytes = (u32)used;
    if (p->arena_span_bytes > p->arena_span_high_bytes)
        p->arena_span_high_bytes = p->arena_span_bytes;
    if (p->arena_span_count > p->arena_span_high_count)
        p->arena_span_high_count = p->arena_span_count;
}

static void proc_span_reset(u32 slot)
{
    if (slot >= MAX_PROCS_PER_CORE)
        return;
    for (u32 i = 0; i < PROC_SPANS_PER_PROCESS; i++)
        proc_spans[slot][i].used = false;
    proc_span_floor[slot] = procs[slot].arena_limit;
    procs[slot].arena_span_bytes = 0;
    procs[slot].arena_span_count = 0;
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
            p->arena_high_bytes = launch_req[uc].migrate_arena_high_bytes;
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
static i32   sys_park(void);
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
static void *sys_span_rent(u32 bytes, u32 align, u32 type);
static i32   sys_span_release(void *ptr);
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
static i32   sys_ipc_fifo_send_span(i32 channel_id, const void *addr, u32 len, u32 flags, u64 tag);
static i32   sys_ipc_fifo_recv(i32 channel_id, void *out, u32 out_max);
static i32   sys_ipc_shm_create(const char *name, u32 peer_principal, u32 owner_acl,
                                u32 peer_acl, u32 size);
static i32   sys_ipc_shm_open(const char *name, u32 want_acl);
static i32   sys_ipc_shm_map(i32 region_id, u32 flags, void **addr_out, u32 *size_out);
static i32   sys_ipc_shm_unmap(i32 map_handle);
static i32   sys_sw_int_kernel(i32 channel_id, u32 event_type, u32 flags);
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
static void  proc_handle_bench_echo(void);
static void  proc_note_desched(u32 reason);
static void  proc_park_note(u32 which);

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
    .park            = sys_park,
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
    .span_rent       = sys_span_rent,
    .span_release    = sys_span_release,
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
    .ipc_fifo_send_span = sys_ipc_fifo_send_span,
    .ipc_fifo_recv   = sys_ipc_fifo_recv,
    .ipc_shm_create  = sys_ipc_shm_create,
    .ipc_shm_open    = sys_ipc_shm_open,
    .ipc_shm_map     = sys_ipc_shm_map,
    .ipc_shm_unmap   = sys_ipc_shm_unmap,
    .sw_int_kernel   = sys_sw_int_kernel,
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

static void proc_handle_bench_echo(void)
{
    struct fifo_msg batch[16];
    struct fifo_span_msg spans[16];
    if (!on_user_core())
        return;
    u32 sn = fifo_span_pop_batch(core_id(), CORE_NET, spans, 16);
    if (sn) {
        struct fifo_span_msg done = spans[sn - 1];
        done.len = sn;
        done.aux = PROC_IPC_OK;
        fifo_span_push_batch(core_id(), CORE_NET, &done, 1);
    }
    struct fifo_msg msg;
    while (fifo_peek(core_id(), CORE_NET, &msg) &&
           (msg.type == MSG_BENCH_ECHO || msg.type == MSG_BENCH_BATCH)) {
        if (!fifo_pop(core_id(), CORE_NET, &msg))
            break;
        if (msg.type == MSG_BENCH_BATCH) {
            u32 want = msg.length > 16U ? 16U : msg.length;
            if (want > 1)
                (void)fifo_pop_batch(core_id(), CORE_NET, batch, want - 1U);
        }
        msg.type = MSG_ACK;
        msg.status = 0;
        fifo_push(core_id(), CORE_NET, &msg);
    }
}

/* Initialise single-instance shared process-table state exactly once. Called by
 * core 0 before it launches the secondary cores so there is no race; the guard
 * makes a later proc_init() from a secondary a no-op for shared state. */
void proc_init_shared(void)
{
    if (initialized)
        return;
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
    dmb_ish();
    initialized = true;
}

void proc_init(void)
{
    u32 uc = on_user_core() ? user_core_slot() : 0;
    if (on_user_core())
        core_mark_online(core_id(), 0x20);
    /* Shared process-table state is single-instance (procs[], current_proc,
     * rr_cursor, next_pid, paged IO, integrity baseline). It MUST be initialised
     * exactly once. Core 0 runs proc_init_shared() before launching the secondary
     * cores, so `initialized` is already true by the time cores 1-3 arrive here;
     * without this guard each secondary re-zeroed procs[] and a late core (e.g.
     * core 3) wiped a process another core (core 2 / httpd) had just created. */
    proc_init_shared();
    if (on_user_core())
        core_mark_online(core_id(), 0x25);
    if (on_user_core()) {
        core_mark_online(core_id(), 0x250);
        preempt_enabled[uc] = false;
        preempt_armed[uc] = false;
        preempt_pending[uc] = false;
        preempt_quantum_ticks[uc] = 1;
        preempt_count_core[uc] = 0;
        core_mark_online(core_id(), 0x251);
        sched_start_ticks_core[uc] = 0;
        sched_idle_ticks_core[uc] = 0;
        sched_idle_enter_ticks_core[uc] = 0;
        sched_idle_count_core[uc] = 0;
        sched_wake_count_core[uc] = 0;
        soft_event_count_core[uc] = 0;
        soft_boost_count_core[uc] = 0;
        /* BSS is already cleared by core0 before secondary cores launch.
         * Do not bulk-clear shared IPC/request namespaces concurrently from
         * secondary cores during bring-up; keep per-core scheduler counters
         * local here and let create/open paths initialise entries explicitly. */
        core_mark_online(core_id(), 0x252);
        core_mark_online(core_id(), 0x26);
    }
    if (on_user_core())
        core_mark_online(core_id(), 0x27);
    ksem_init_core();
    if (on_user_core())
        core_mark_online(core_id(), 0x28);
    workq_init_core();
    if (on_user_core())
        core_mark_online(core_id(), 0x29);
    mmu_switch_to_kernel();
    if (on_user_core())
        core_mark_online(core_id(), 0x2A);
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

    struct proc_image_validation image;
    if (!proc_validate_image_path(path, &image)) {
        uart_puts("[proc] image validate failed status=");
        uart_puts(proc_image_status_name(image.status));
        uart_puts(" path=");
        uart_puts(path);
        uart_putc('\n');
        return -1;
    }
    if (image.launch_mode != PROC_IMAGE_LAUNCH_FLAT_DIRECT) {
        uart_puts("[proc] launch blocked format=");
        uart_puts(proc_image_format_name(image.format));
        uart_puts(" launch=");
        uart_puts(proc_image_launch_mode_name(image.launch_mode));
        uart_puts(" path=");
        uart_puts(path);
        uart_putc('\n');
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
    proc_wake_pending[slot] = 0;
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
    p->entry_pc = (u64)(usize)base;
    p->arena_base = ((u64)(usize)base + loaded + L3_PAGE_SIZE - 1) & ~(L3_PAGE_SIZE - 1);
    p->arena_limit = (u64)(usize)base + PROC_SLOT_SIZE - 65536ULL;
    if (p->arena_base >= p->arena_limit) {
        uart_puts("[proc] image leaves no data arena\n");
        p->state = PROC_EMPTY;
        p->pid = 0;
        return -1;
    }
    p->arena_capacity_bytes = (p->arena_limit > p->arena_base) ?
                              (u32)(p->arena_limit - p->arena_base) : 0;
    p->arena_high_bytes = 0;
    p->arena_span_bytes = 0;
    p->arena_span_high_bytes = 0;
    p->arena_span_count = 0;
    p->arena_span_high_count = 0;
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

    heap_top[(u32)slot] = p->arena_base;
    proc_span_reset((u32)slot);
    dcache_clean_range((u64)(usize)base, loaded);
    icache_invalidate_range((u64)(usize)base, loaded);

    p->entry_sp = (u64)(usize)(base + PROC_SLOT_SIZE - 16);
    p->entry_spsr = PROC_ENTRY_SPSR_EL0_DAIF;
    p->entry_flags = proc_entry_contract_flags();
    if (!proc_entry_contract_validate(p)) {
        uart_puts("[proc] invalid entry contract\n");
        p->state = PROC_EMPTY;
        p->pid = 0;
        return -1;
    }

    simd_zero(&p->ctx, sizeof(p->ctx));
    p->ctx.x19_x30[0] = (u64)(usize)&kernel_api_tab;  /* x19 */
    p->ctx.x19_x30[1] = p->entry_pc;                /* x20 */
    p->ctx.x19_x30[11] = (u64)(usize)proc_trampoline; /* x30 = LR */
    p->ctx.sp = p->entry_sp;

    if (!mmu_user_table_build_split(core_id(), (u32)slot, (u64)(usize)base, PROC_SLOT_SIZE, loaded)) {
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

/* Launch a flat binary from an in-memory buffer (no WALFS). Used for userland
 * apps embedded in the kernel image. The blob must be a flat binary linked at
 * slot_base() for the chosen slot; runs as PRINCIPAL_ROOT (trusted kernel
 * launch). Must be called on the target core. Returns pid or -1. */
i32 proc_exec_from_mem(const char *name, const u8 *blob, u32 blob_len,
                       u32 priority_class, u32 affinity_core)
{
    if (!initialized)
        return -1;
    if (!proc_prio_valid(priority_class))
        return -1;
    if (affinity_core != core_id())
        return -1;
    if (!blob || blob_len == 0 || blob_len > PROC_SLOT_SIZE - 64U)
        return -1;

    i32 slot = find_empty_slot();
    if (slot < 0) {
        uart_puts("[proc] mem-exec: no free slot\n");
        return -1;
    }

    u8 *base = slot_base((u32)slot);
    /* The blob is linked at a fixed slot base; refuse if this slot doesn't
     * match (would mean absolute relocations land at the wrong address). */
    if ((u64)(usize)base != PROC_EMBED_BASE) {
        uart_puts("[proc] mem-exec: slot base mismatch\n");
        return -1;
    }
    dma_zero(5, base, PROC_SLOT_SIZE);
    simd_memcpy(base, blob, blob_len);
    u32 loaded = blob_len;
    u32 exec_hash = hw_crc32c(base, loaded);

    struct process *p = &procs[slot];
    p->pid = next_pid++;
    p->parent_pid = 0;
    p->state = PROC_READY;
    proc_wake_pending[slot] = 0;
    p->principal_id = PRINCIPAL_ROOT;   /* trusted: embedded in kernel image */
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
    p->entry_pc = (u64)(usize)base;
    p->arena_base = ((u64)(usize)base + loaded + L3_PAGE_SIZE - 1) & ~(L3_PAGE_SIZE - 1);
    p->arena_limit = (u64)(usize)base + PROC_SLOT_SIZE - 65536ULL;
    if (p->arena_base >= p->arena_limit) {
        uart_puts("[proc] mem-exec: no data arena\n");
        p->state = PROC_EMPTY;
        p->pid = 0;
        return -1;
    }
    p->arena_capacity_bytes = (u32)(p->arena_limit - p->arena_base);
    p->arena_high_bytes = 0;
    p->arena_span_bytes = 0;
    p->arena_span_high_bytes = 0;
    p->arena_span_count = 0;
    p->arena_span_high_count = 0;
    p->image_path[0] = 0;
    for (u32 pi = 0; name && pi + 1 < sizeof(p->image_path) && name[pi]; pi++) {
        p->image_path[pi] = name[pi];
        p->image_path[pi + 1] = 0;
    }
    p->capsule_enabled = false;
    p->capsule_manifest_hash = 0;

    heap_top[(u32)slot] = p->arena_base;
    proc_span_reset((u32)slot);
    dcache_clean_range((u64)(usize)base, loaded);
    icache_invalidate_range((u64)(usize)base, loaded);

    p->entry_sp = (u64)(usize)(base + PROC_SLOT_SIZE - 16);
    p->entry_spsr = PROC_ENTRY_SPSR_EL0_DAIF;
    p->entry_flags = proc_entry_contract_flags();
    if (!proc_entry_contract_validate(p)) {
        uart_puts("[proc] mem-exec: bad entry contract\n");
        p->state = PROC_EMPTY;
        p->pid = 0;
        return -1;
    }

    simd_zero(&p->ctx, sizeof(p->ctx));
    p->ctx.x19_x30[0] = (u64)(usize)&kernel_api_tab;  /* x19 */
    p->ctx.x19_x30[1] = p->entry_pc;                  /* x20 */
    p->ctx.x19_x30[11] = (u64)(usize)proc_trampoline; /* x30 = LR */
    p->ctx.sp = p->entry_sp;

    if (!mmu_user_table_build_split(core_id(), (u32)slot, (u64)(usize)base,
                                    PROC_SLOT_SIZE, loaded)) {
        p->state = PROC_EMPTY;
        p->pid = 0;
        return -1;
    }
    /* Map the shared IPC_SHM window so the process can reach the kernel<->user
     * HTTP bridge (and any other shared-memory IPC) at IPC_SHM_BASE. */
    (void)mmu_user_ipc_shm_window(core_id(), (u32)slot, true);

    uart_puts("[proc] mem-exec pid=");
    uart_hex(p->pid);
    uart_puts(" name=");
    uart_puts(name ? name : "?");
    uart_putc('\n');

    return (i32)p->pid;
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
    proc_note_desched(2U);
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
    proc_note_desched(4U);
    ctx_switch(&p->ctx, &scheduler_ctx);
    __builtin_unreachable();
}

void proc_schedule(void)
{
    bool user = on_user_core();
    u32 uc = user ? user_core_slot() : 0;
    if (!initialized)
        proc_init();

    if (user) {
        core_mark_online(core_id(), 6);
        sched_start_ticks_core[uc] = proc_sched_counter_ticks();
    } else {
        uart_puts("[proc] scheduler running on core ");
        uart_hex(core_id());
        uart_putc('\n');
    }

    for (;;) {
        watchdog_touch(core_id());
        workq_drain(8);
        if (cohdiag_consumer_arm)
            proc_cohdiag_consumer_tick();
        proc_drain_remote_wakes();
        proc_sched_heartbeat();
        proc_sched_stage(10);
        proc_handle_bench_echo();
        proc_sched_stage(11);
        proc_handle_launch_request();
        proc_sched_stage(12);
        bool found = false;
        for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
            /* Only reap processes homed to this core. procs[] is shared across
             * the per-core schedulers; without this gate one core could reap a
             * DEAD slot another core still owns. */
            if (procs[i].affinity_core != core_id())
                continue;
            if (procs[i].state == PROC_DEAD) {
                if (procs[i].pid != 0) {
                    u64 out = 0;
                    (void)el2_hvc_call(EL2_HVC_PORT_UNBIND_ALL, procs[i].pid, 0, 0, 0, &out);
                }
                procs[i].state = PROC_EMPTY;
                procs[i].pid = 0;
            }
        }
        proc_sched_stage(13);

        bool has_non_lazy_ready = false;
        for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
            if (procs[i].affinity_core != core_id())
                continue;
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
            /* Dispatch only processes homed to this core. Each core builds its
             * OWN per-core user page table for a slot (mmu_user_table_build_split
             * runs on the creating core); a non-owner core would fail
             * mmu_switch_to_user and wrongly mark the process DEAD. */
            if (procs[i].affinity_core != core_id())
                continue;
            if (has_non_lazy_ready && procs[i].priority_class == PROC_PRIO_LAZY)
                continue;
            if (chosen == 0xFFFFFFFFU || procs[i].priority_class > best_prio) {
                chosen = i;
                best_prio = procs[i].priority_class;
            }
        }
        proc_sched_stage(14);

        if (chosen != 0xFFFFFFFFU) {
            found = true;
            proc_sched_stage(20);
            if (!proc_integrity_maybe_check(chosen))
                continue;
            proc_sched_stage(30);
            rr_cursor = chosen;
            current_proc = chosen;
            procs[chosen].state = PROC_RUNNING;
            proc_diag_note_dispatch();
            procs[chosen].ticks = timer_ticks();
            principal_set_current(procs[chosen].principal_id);
            proc_sched_stage(35);
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
            proc_sched_stage(40);
            if (!mmu_switch_to_user(core_id(), chosen)) {
                procs[chosen].state = PROC_DEAD;
                procs[chosen].exit_code = 0xFFFF0002U;
                mmu_switch_to_kernel();
                principal_set_current(PRINCIPAL_ROOT);
                (void)el2_stage2_activate(PROC_CAPSULE_ID_NONE);
            } else {
                proc_sched_stage(50);
                if (user && preempt_enabled[uc]) {
                    preempt_pending[uc] = false;
                    preempt_armed[uc] = true;
                }
                proc_sched_note_ctx_enter(procs[chosen].pid);
                ctx_switch(&scheduler_ctx, &procs[chosen].ctx);
                proc_sched_note_ctx_exit();
                proc_sched_stage(51);
                if (user) {
                    preempt_armed[uc] = false;
                    preempt_pending[uc] = false;
                }
                mmu_switch_to_kernel();
                proc_sched_stage(52);
                principal_set_current(PRINCIPAL_ROOT);
                (void)el2_stage2_activate(PROC_CAPSULE_ID_NONE);
                proc_sched_stage(53);
            }
        }

        if (!found) {
            workq_drain(8);
            if (user) {
                core_mark_online(core_id(), 0x80);
                sched_idle_count_core[uc]++;
                proc_diag_note_wfe();
                u64 idle_start = proc_sched_counter_ticks();
                sched_idle_enter_ticks_core[uc] = idle_start;
                proc_sched_stage(60);
                wfe();
                proc_sched_stage(61);
                u64 idle_end = proc_sched_counter_ticks();
                sched_idle_enter_ticks_core[uc] = 0;
                if (idle_end >= idle_start)
                    sched_idle_ticks_core[uc] += idle_end - idle_start;
                sched_wake_count_core[uc]++;
                core_mark_online(core_id(), 0x81);
            } else {
                wfe();
            }
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

    /* Self-waking idle scheduler: enable the architected timer event stream so
     * WFE on this core wakes at a bounded rate (~every 2^EVNTI counter ticks)
     * even when no interrupt or cross-core SEV arrives. Empirically, on this
     * bring-up a remote SEV from core 0 (the network core posting a process
     * wake) does NOT reliably wake a user core parked in WFE — the wake can be
     * missed and the parked process would sleep forever. (NOTE: this is NOT the
     * A76 "SMPEN" bit — SMPEN is a no-op on A76, which is always coherent for
     * inner-shareable accesses. The true cause is the same unresolved cross-core
     * cacheable-visibility issue documented at the wake-ring struct below and in
     * uhttp_bridge.h — under investigation.) With the event stream on, the idle
     * loop re-drains the remote wake ring and re-checks runnable processes on
     * every event tick; SEV remains a best-effort fast-path wake when it lands.
     * EVNTI=8 selects CNTVCT bit 8 → an event roughly every 256 ticks. */
    {
        u64 cntkctl;
        __asm__ volatile("mrs %0, cntkctl_el1" : "=r"(cntkctl));
        cntkctl &= ~((0xFULL << 4) | (1ULL << 3)); /* clear EVNTI[7:4], EVNTDIR */
        cntkctl |= (8ULL << 4);                    /* EVNTI = CNTVCT bit 8 */
        cntkctl |= (1ULL << 2);                    /* EVNTEN = 1 */
        __asm__ volatile("msr cntkctl_el1, %0" :: "r"(cntkctl));
        isb();
    }

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
    proc_note_desched(3U);
    frame->x[30] = frame->elr;
    frame->elr = (u64)(usize)proc_preempt_trampoline;
}

u64 proc_preemptions(void)
{
    if (!on_user_core())
        return 0;
    return preempt_count_core[user_core_slot()];
}

/* Coherent wake-path diagnostics (defined after PROC_RWAKE_SHARED below). */
static void proc_rwake_note_soft(u32 pid, i32 slot, u32 state_before);

bool proc_soft_event(u32 target_pid, u32 event_type, bool boost)
{
    if (!initialized || !on_user_core() || target_pid == 0)
        return false;
    u32 uc = user_core_slot();
    soft_event_count_core[uc]++;
    (void)event_type;

    i32 slot = proc_find_slot_by_pid(target_pid);
    proc_rwake_note_soft(target_pid, slot,
                         (slot >= 0) ? procs[(u32)slot].state : 0xFFFFFFFFU);
    if (slot < 0)
        return false;

    struct process *p = &procs[(u32)slot];
    /* Latch the wake STICKILY before touching state. The old code only flipped
     * BLOCKED->READY, dropping the wake whenever the target had not yet committed
     * to BLOCKED (still RUNNING/READY mid park-transition) — the port-81 504
     * strand. proc_park() consumes this latch and refuses to block while it is
     * set, so a wake can never be lost regardless of the park/wake interleaving. */
    proc_wake_pending[(u32)slot] = 1U;
    if (p->state == PROC_BLOCKED)
        p->state = PROC_READY;
    if (boost && (p->state == PROC_READY || p->state == PROC_RUNNING)) {
        rr_cursor = ((u32)slot + MAX_PROCS_PER_CORE - 1U) % MAX_PROCS_PER_CORE;
        soft_boost_count_core[uc]++;
    }
    sev();
    return true;
}

/* ── Cross-core process wake ──
 * Any core (notably core 0, the network core, which is NOT a user core and so
 * cannot call proc_soft_event) posts a target pid onto the target user core's
 * wake ring and signals SEV. The target core's scheduler drains the ring and
 * runs proc_soft_event locally, flipping the parked process READY. The wake is
 * queued (not delivered inline), so there is no lost-wakeup window: the
 * scheduler always drains the ring after a process parks and before it WFEs. */
#define PROC_RWAKE_DEPTH 64U
struct proc_rwake_ring {
    volatile u32 head;             /* posting core writes; target core reads */
    u32 _pad_head[15];             /* isolate head on its own 64-byte line */
    volatile u32 tail;             /* target core writes; posting core reads */
    u32 _pad_tail[15];             /* isolate tail on its own line */
    u32 pids[PROC_RWAKE_DEPTH];    /* posting core writes; target core reads */
};

/* The wake rings + counters MUST live in the SHARED_FIFO region with EXPLICIT
 * cache maintenance, NOT in plain kernel .bss. Empirically, a single low-rate
 * cacheable .bss write by core 0 is not reliably visible to a user core's drain —
 * the posted wake is silently lost and the parked process never runs (observed:
 * wake_posted=1, wake_drained=0, httpd stuck blocked). NOTE: despite EVERY core
 * sharing identical kernel mappings (shared_ttbr0/shared_mair/shared_tcr in
 * mmu.c, Inner-Shareable WB PTEs), cross-core cacheable visibility does NOT behave
 * coherently on this bring-up. This is NOT the A76 "SMPEN" bit (a no-op on A76 —
 * the core is always coherent for inner-shareable accesses); the real cause is
 * unresolved and under investigation — it is also NOT EL2 stage-2 (HCR_EL2.VM is
 * cleared for non-capsule processes, so httpd/core-0 run with stage-1 only), and
 * it is NOT an attribute mismatch (every core shares shared_ttbr0/shared_mair/
 * shared_tcr). It correlates with anomalously slow ~118ns cacheable writes
 * (membench), hinting the data cache may not be retaining/snooping these lines as
 * expected. The proven mitigation is the SHARED_FIFO region with explicit dc cvac/civac + barriers
 * (the lock-free FIFOs and the uhttp bridge cross cores from there reliably). Park
 * the rings in its top page, which fifo_init_all() zero-clears at boot before any
 * secondary core launches. */
struct proc_rwake_shared {
    struct proc_rwake_ring ring[4];
    /* core0-written counters (one line). */
    volatile u32 posted;
    volatile u32 full;
    u32 _pad_c0[14];
    /* target-core-written counter, isolated so core0 can dc-ivac it for an
     * accurate read without discarding a posted/full it owns. */
    volatile u32 drained;
    u32 _pad_c1[15];
    volatile u32 loops[4];     /* per-core scheduler-loop heartbeat (liveness probe) */
    volatile u32 ctx_enter[4]; /* per-core: count of ctx_switch INTO a process */
    volatile u32 ctx_exit[4];  /* per-core: count of ctx_switch returns FROM a process */
    volatile u32 last_pid[4];  /* per-core: pid of the most recent ctx_switch target */
    volatile u32 stage[4];     /* per-core: last scheduler dispatch stage reached */
    /* one-off firehose wake-path debug: written by the user core's drain /
     * soft_event, read by core 0. NC memory, single-writer-per-field. */
    volatile u32 dbg_iters;    /* drain loop bodies executed (== drained) */
    volatile u32 dbg_pid;      /* last pid value read from pids[] in drain */
    volatile u32 dbg_zero;     /* drain reads where pid == 0 */
    volatile u32 dbg_calls;    /* soft_event entries reaching find_slot */
    volatile u32 dbg_noslot;   /* soft_event find_slot < 0 count */
    volatile u32 dbg_state;    /* p->state soft_event last saw for the slot */
    volatile u32 live_state[4];/* per-core: procs[current_proc].state, published each sched loop */
    volatile u32 live_pid[4];  /* per-core: procs[current_proc].pid, published each sched loop */
    volatile u32 disp_cnt[4];  /* per-core: count of dispatches (RUNNING set, proc.c:1895) */
    volatile u32 wfe_cnt[4];   /* per-core: scheduler WFE sleeps taken (found==false idle path) */
    /* port-81 504 root-cause trace. desched_reason[c] records HOW the last
     * process left core c (1=park-block, 2=yield, 3=preempt, 4=exit). The drain
     * re-readies any target found off-core in RUNNING and counts it in
     * dbg_rescued. park_* break down proc_park's path selection. */
    volatile u32 dbg_rescued;
    volatile u32 desched_reason[4];
    volatile u32 park_enter;
    volatile u32 park_early;
    volatile u32 park_block;
    volatile u32 park_resume;
    u32 _pad_dbg[1];
};
#define PROC_RWAKE_SHARED \
    ((volatile struct proc_rwake_shared *)(SHARED_FIFO_BASE + SHARED_FIFO_SIZE - 0x1000UL))

_Static_assert((SHARED_FIFO_SIZE - 0x1000UL) >=
                   (16UL * sizeof(struct fifo) + 16UL * sizeof(struct fifo_span)),
               "proc_rwake shared page overlaps the FIFO pool");
_Static_assert(sizeof(struct proc_rwake_shared) <= 0x1000UL,
               "proc_rwake shared struct exceeds its page");

static inline u64 proc_irq_save(void) {
    u64 daif;
    __asm__ volatile("mrs %0, daif" : "=r"(daif));
    __asm__ volatile("msr daifset, #2" ::: "memory");
    return daif;
}
static inline void proc_irq_restore(u64 daif) {
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
}

bool proc_post_remote_wake(u32 target_core, u32 pid) {
    if (target_core >= 4U || pid == 0)
        return false;
    volatile struct proc_rwake_ring *r = &PROC_RWAKE_SHARED->ring[target_core];
    u64 daif = proc_irq_save();
    u32 head = r->head;        /* this core's own prior store: cached read is fine */
    /* tail is advanced by the target core; refresh before the ring-full check. */
    dcache_invalidate_range((u64)(usize)&r->tail, sizeof(r->tail));
    u32 next = (head + 1U) & (PROC_RWAKE_DEPTH - 1U);
    if (next == r->tail) {
        PROC_RWAKE_SHARED->full++;
        proc_irq_restore(daif);
        return false;          /* ring full: receiver will catch up on next poll */
    }
    r->pids[head] = pid;
    dcache_clean_range((u64)(usize)&r->pids[head], sizeof(r->pids[head]));
    r->head = next;
    dcache_clean_range((u64)(usize)&r->head, sizeof(r->head));
    PROC_RWAKE_SHARED->posted++;
    dsb_ishst();               /* head globally visible before the wake event */
    proc_irq_restore(daif);
    sev();
    return true;
}

void proc_rwake_stats(u32 *posted, u32 *drained, u32 *full)
{
    /* drained is written by target cores; refresh core 0's copy before reading. */
    dcache_invalidate_range((u64)(usize)&PROC_RWAKE_SHARED->drained, sizeof(u32));
    if (posted)  *posted  = PROC_RWAKE_SHARED->posted;
    if (drained) *drained = PROC_RWAKE_SHARED->drained;
    if (full)    *full    = PROC_RWAKE_SHARED->full;
}

/* NC-coherent single-writer store for the wake-path debug counters. */
static inline void rwake_dbg_push(volatile u32 *p, u32 v) {
    *p = v;
    __asm__ volatile("dc cvac, %0" :: "r"(p) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
}

static void proc_rwake_note_soft(u32 pid, i32 slot, u32 state_before) {
    (void)pid;
    rwake_dbg_push(&PROC_RWAKE_SHARED->dbg_calls, PROC_RWAKE_SHARED->dbg_calls + 1U);
    if (slot < 0)
        rwake_dbg_push(&PROC_RWAKE_SHARED->dbg_noslot, PROC_RWAKE_SHARED->dbg_noslot + 1U);
    else
        rwake_dbg_push(&PROC_RWAKE_SHARED->dbg_state, state_before);
}

/* Publish the reason the current process last left its core (port-81 504 trace).
 * NC single-writer per core, so core 0 can read it without a coherency race. */
static void proc_note_desched(u32 reason) {
    u32 c = core_id() & 3U;
    rwake_dbg_push(&PROC_RWAKE_SHARED->desched_reason[c], reason);
}

/* proc_park() path counters. which: 0=enter, 1=early-return, 2=block, 3=resume. */
static void proc_park_note(u32 which) {
    switch (which) {
    case 0U: rwake_dbg_push(&PROC_RWAKE_SHARED->park_enter,
                            PROC_RWAKE_SHARED->park_enter + 1U); break;
    case 1U: rwake_dbg_push(&PROC_RWAKE_SHARED->park_early,
                            PROC_RWAKE_SHARED->park_early + 1U); break;
    case 2U: rwake_dbg_push(&PROC_RWAKE_SHARED->park_block,
                            PROC_RWAKE_SHARED->park_block + 1U); break;
    default: rwake_dbg_push(&PROC_RWAKE_SHARED->park_resume,
                            PROC_RWAKE_SHARED->park_resume + 1U); break;
    }
}

static void proc_drain_remote_wakes(void) {
    if (!on_user_core())
        return;
    volatile struct proc_rwake_ring *r = &PROC_RWAKE_SHARED->ring[core_id() & 3U];
    /* head + pids are written by the posting core; drop our stale copies so a
     * single low-frequency post by an otherwise-idle core 0 is observed. */
    dcache_invalidate_range((u64)(usize)&r->head, sizeof(r->head));
    while (r->tail != r->head) {
        dcache_invalidate_range((u64)(usize)&r->pids[r->tail], sizeof(r->pids[0]));
        u32 pid = r->pids[r->tail];
        rwake_dbg_push(&PROC_RWAKE_SHARED->dbg_pid, pid);
        rwake_dbg_push(&PROC_RWAKE_SHARED->dbg_iters, PROC_RWAKE_SHARED->dbg_iters + 1U);
        if (pid == 0)
            rwake_dbg_push(&PROC_RWAKE_SHARED->dbg_zero, PROC_RWAKE_SHARED->dbg_zero + 1U);
        r->tail = (r->tail + 1U) & (PROC_RWAKE_DEPTH - 1U);
        dcache_clean_range((u64)(usize)&r->tail, sizeof(r->tail));  /* tail -> PoC */
        PROC_RWAKE_SHARED->drained++;
        dcache_clean_range((u64)(usize)&PROC_RWAKE_SHARED->drained, sizeof(u32));
        (void)proc_soft_event(pid, PROC_SOFT_EVENT_IPC_FIFO, true);
        /* ROOT-CAUSE FIX for the port-81 504: proc_soft_event() only re-readies a
         * BLOCKED target. A process that descheduled while still marked RUNNING
         * (the off-core RUNNING state proven by d_state=2) is otherwise skipped by
         * the READY-only dispatch scan forever, so httpd serves req#1 then never
         * wakes again. We run here in the scheduler loop with NO user process
         * on-core (documented proc_soft_event invariant: it never runs while the
         * target is on-core), so a RUNNING reading is provably stale/stuck and is
         * safe to force back to READY for re-dispatch. */
        {
            i32 rs = proc_find_slot_by_pid(pid);
            if (rs >= 0 && procs[(u32)rs].state == PROC_RUNNING) {
                procs[(u32)rs].state = PROC_READY;
                rr_cursor = ((u32)rs + MAX_PROCS_PER_CORE - 1U) % MAX_PROCS_PER_CORE;
                rwake_dbg_push(&PROC_RWAKE_SHARED->dbg_rescued,
                               PROC_RWAKE_SHARED->dbg_rescued + 1U);
            }
        }
        dcache_invalidate_range((u64)(usize)&r->head, sizeof(r->head)); /* re-check head */
    }
}

static void proc_diag_note_dispatch(void) {
    u32 c = core_id() & 3U;
    PROC_RWAKE_SHARED->disp_cnt[c]++;
    __asm__ volatile("dc cvac, %0" :: "r"(&PROC_RWAKE_SHARED->disp_cnt[c]) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
}

static void proc_diag_note_wfe(void) {
    u32 c = core_id() & 3U;
    PROC_RWAKE_SHARED->wfe_cnt[c]++;
    __asm__ volatile("dc cvac, %0" :: "r"(&PROC_RWAKE_SHARED->wfe_cnt[c]) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
}

/* Per-core scheduler-loop heartbeat, in coherent shared RAM so any core can read
 * another core's liveness. A stalled counter means that core is wedged (e.g.
 * parked in WFE with no wake source) rather than merely not seeing a wake. */
static void proc_sched_heartbeat(void) {
    u32 c = core_id() & 3U;
    PROC_RWAKE_SHARED->loops[c]++;
    __asm__ volatile("dc cvac, %0" :: "r"(&PROC_RWAKE_SHARED->loops[c]) : "memory");
    /* Publish the LIVE state/pid of this core's current process (cleaned to PoC)
     * so core 0 reads the real scheduler state, not the soft_event snapshot. */
    {
        u32 slot = current_proc;
        u32 st = 0, pid = 0;
        if (slot < MAX_PROCS_PER_CORE) {
            st = procs[slot].state;
            pid = procs[slot].pid;
        }
        PROC_RWAKE_SHARED->live_state[c] = st;
        PROC_RWAKE_SHARED->live_pid[c] = pid;
        __asm__ volatile("dc cvac, %0" :: "r"(&PROC_RWAKE_SHARED->live_state[c]) : "memory");
        __asm__ volatile("dc cvac, %0" :: "r"(&PROC_RWAKE_SHARED->live_pid[c]) : "memory");
    }
    __asm__ volatile("dsb ish" ::: "memory");
}

/* Record entry into a process context switch (before ctx_switch) and exit
 * (after it returns). If, for some core, ctx_enter advances but ctx_exit does
 * not, that core is wedged INSIDE the switched-to process — not idle, not in the
 * scheduler. last_pid identifies which process it dived into. */
static void proc_sched_note_ctx_enter(u32 pid) {
    u32 c = core_id() & 3U;
    PROC_RWAKE_SHARED->last_pid[c] = pid;
    PROC_RWAKE_SHARED->ctx_enter[c]++;
    __asm__ volatile("dc cvac, %0" :: "r"(&PROC_RWAKE_SHARED->ctx_enter[c]) : "memory");
    __asm__ volatile("dc cvac, %0" :: "r"(&PROC_RWAKE_SHARED->last_pid[c]) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
}

static void proc_sched_note_ctx_exit(void) {
    u32 c = core_id() & 3U;
    PROC_RWAKE_SHARED->ctx_exit[c]++;
    __asm__ volatile("dc cvac, %0" :: "r"(&PROC_RWAKE_SHARED->ctx_exit[c]) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
}

static void proc_sched_stage(u32 s) {
    u32 c = core_id() & 3U;
    PROC_RWAKE_SHARED->stage[c] = s;
    __asm__ volatile("dc cvac, %0" :: "r"(&PROC_RWAKE_SHARED->stage[c]) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
}

/* Reader-side coherency: these diagnostic fields are written ONLY by user cores
 * (never by core 0). Invalidate core 0's possibly-stale cached copy to PoC before
 * reading so a single write by a now-sleeping core is observed even if snoop
 * coherency is not active. Safe because core 0 holds no dirty data here. */
static inline void diag_inval_word(volatile void *p) {
    __asm__ volatile("dc ivac, %0" :: "r"(p) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
}

u32 proc_sched_loops(u32 core) {
    if (core >= 4U)
        return 0;
    diag_inval_word(&PROC_RWAKE_SHARED->loops[core]);
    return PROC_RWAKE_SHARED->loops[core];
}

void proc_sched_ctx_stats(u32 core, u32 *enter, u32 *exit, u32 *last_pid) {
    if (core >= 4U) {
        if (enter) *enter = 0;
        if (exit) *exit = 0;
        if (last_pid) *last_pid = 0;
        return;
    }
    diag_inval_word(&PROC_RWAKE_SHARED->ctx_enter[core]);
    diag_inval_word(&PROC_RWAKE_SHARED->ctx_exit[core]);
    diag_inval_word(&PROC_RWAKE_SHARED->last_pid[core]);
    if (enter) *enter = PROC_RWAKE_SHARED->ctx_enter[core];
    if (exit) *exit = PROC_RWAKE_SHARED->ctx_exit[core];
    if (last_pid) *last_pid = PROC_RWAKE_SHARED->last_pid[core];
}

u32 proc_sched_stage_get(u32 core) {
    if (core >= 4U)
        return 0;
    diag_inval_word(&PROC_RWAKE_SHARED->stage[core]);
    return PROC_RWAKE_SHARED->stage[core];
}

void proc_rwake_dbg(u32 *iters, u32 *pid, u32 *zero, u32 *calls, u32 *noslot, u32 *state) {
    diag_inval_word(&PROC_RWAKE_SHARED->dbg_iters);
    diag_inval_word(&PROC_RWAKE_SHARED->dbg_pid);
    diag_inval_word(&PROC_RWAKE_SHARED->dbg_zero);
    diag_inval_word(&PROC_RWAKE_SHARED->dbg_calls);
    diag_inval_word(&PROC_RWAKE_SHARED->dbg_noslot);
    diag_inval_word(&PROC_RWAKE_SHARED->dbg_state);
    if (iters)  *iters  = PROC_RWAKE_SHARED->dbg_iters;
    if (pid)    *pid    = PROC_RWAKE_SHARED->dbg_pid;
    if (zero)   *zero   = PROC_RWAKE_SHARED->dbg_zero;
    if (calls)  *calls  = PROC_RWAKE_SHARED->dbg_calls;
    if (noslot) *noslot = PROC_RWAKE_SHARED->dbg_noslot;
    if (state)  *state  = PROC_RWAKE_SHARED->dbg_state;
}

/* port-81 504 trace readout: rescued count + per-core last-deschedule reason +
 * proc_park path counters. Core 0 invalidates before reading the NC fields. */
void proc_rwake_park_dbg(u32 core, u32 *rescued, u32 *reason,
                         u32 *p_enter, u32 *p_early, u32 *p_block, u32 *p_resume) {
    if (core >= 4U)
        core = 0U;
    diag_inval_word(&PROC_RWAKE_SHARED->dbg_rescued);
    diag_inval_word(&PROC_RWAKE_SHARED->desched_reason[core]);
    diag_inval_word(&PROC_RWAKE_SHARED->park_enter);
    diag_inval_word(&PROC_RWAKE_SHARED->park_early);
    diag_inval_word(&PROC_RWAKE_SHARED->park_block);
    diag_inval_word(&PROC_RWAKE_SHARED->park_resume);
    if (rescued)  *rescued  = PROC_RWAKE_SHARED->dbg_rescued;
    if (reason)   *reason   = PROC_RWAKE_SHARED->desched_reason[core];
    if (p_enter)  *p_enter  = PROC_RWAKE_SHARED->park_enter;
    if (p_early)  *p_early  = PROC_RWAKE_SHARED->park_early;
    if (p_block)  *p_block  = PROC_RWAKE_SHARED->park_block;
    if (p_resume) *p_resume = PROC_RWAKE_SHARED->park_resume;
}

/* Live per-core scheduler state, published every scheduler loop and cleaned to
 * PoC (unlike the soft_event d_state SNAPSHOT). Lets core 0 read the REAL
 * current process state, dispatch count, and WFE-sleep count for a user core. */
void proc_rwake_live(u32 core, u32 *live_state, u32 *live_pid, u32 *disp, u32 *wfe) {
    if (core >= 4U) {
        if (live_state) *live_state = 0;
        if (live_pid)   *live_pid   = 0;
        if (disp)       *disp       = 0;
        if (wfe)        *wfe        = 0;
        return;
    }
    diag_inval_word(&PROC_RWAKE_SHARED->live_state[core]);
    diag_inval_word(&PROC_RWAKE_SHARED->live_pid[core]);
    diag_inval_word(&PROC_RWAKE_SHARED->disp_cnt[core]);
    diag_inval_word(&PROC_RWAKE_SHARED->wfe_cnt[core]);
    if (live_state) *live_state = PROC_RWAKE_SHARED->live_state[core];
    if (live_pid)   *live_pid   = PROC_RWAKE_SHARED->live_pid[core];
    if (disp)       *disp       = PROC_RWAKE_SHARED->disp_cnt[core];
    if (wfe)        *wfe        = PROC_RWAKE_SHARED->wfe_cnt[core];
}

/* Block the current process until a soft event (e.g. a remote wake) flips it
 * READY. Mirrors proc_yield but parks the process as BLOCKED so the scheduler
 * runs others / idles in WFE until woken. */
void proc_park(void) {
    bool user = on_user_core();
    u32 uc = user ? user_core_slot() : 0;
    struct process *p = &procs[current_proc];
    proc_park_note(0U); /* park_enter++ */
    /* Disarm preemption FIRST so no timer-preempt can fire between the sticky-wake
     * test and the BLOCKED commit. proc_soft_event() only runs on this same
     * (affinity) core from the wake-ring drain and never while this process is
     * on-core, so disarming preemption makes the test-and-block atomic w.r.t.
     * wake_pending without masking IRQs. */
    if (user) {
        preempt_armed[uc] = false;
        preempt_pending[uc] = false;
    }
    /* A wake delivered after our caller decided "no work" but before we parked
     * leaves wake_pending set even though our state was never BLOCKED. Consume it
     * and return WITHOUT blocking; the caller re-loops and re-checks its work
     * source. This closes the lost-wakeup race behind the port-81 504 strand. */
    if (proc_wake_pending[current_proc]) {
        proc_wake_pending[current_proc] = 0;
        if (user && preempt_enabled[uc])
            preempt_armed[uc] = true;
        proc_park_note(1U); /* park_early++ */
        return;
    }
    proc_account_runtime(p);
    p->state = PROC_BLOCKED;
    proc_park_note(2U); /* park_block++ */
    proc_note_desched(1U);
    ctx_switch(&p->ctx, &scheduler_ctx);
    proc_park_note(3U); /* park_resume++ */
    proc_wake_pending[current_proc] = 0;   /* consume the wake that dispatched us */
    if (user && preempt_enabled[uc] && p->state == PROC_RUNNING)
        preempt_armed[uc] = true;
}

/* Deterministic, seq+index-dependent payload pattern. */
static inline u64 cohdiag_pat(u64 seq, u32 k)
{
    u64 a = (seq + 1ULL) * 0x9E3779B97F4A7C15ULL;
    u64 b = ((u64)k + 1ULL) * 0xD1B54A32D192ED03ULL;
    return a ^ b ^ (seq << (k & 7U)) ^ (seq >> ((64U - (k & 7U)) & 63U));
}

/* Probe the effective stage-1 attributes of a VA via AT S1E1R + PAR_EL1. */
static void cohdiag_probe_attr(u64 addr, u32 *attr, u32 *sh, u32 *fault)
{
    u64 par = 0;
    __asm__ volatile("at s1e1r, %1\n\tisb\n\tmrs %0, par_el1"
                     : "=r"(par) : "r"(addr) : "memory");
    if (par & 1ULL) {                 /* PAR_EL1.F == 1 -> translation fault */
        *fault = 1;
        *attr = 0;
        *sh = 0;
        return;
    }
    *attr = (u32)((par >> 56) & 0xFFULL);
    *sh   = (u32)((par >> 8) & 0x3ULL);
}

/* Consumer body: spin observing seq, acquire-order the payload read, verify. */
static void cohdiag_consumer_run(volatile struct cohdiag_a *a)
{
    u64 last = 0;
    u64 spins = 0;
    u32 use_acq = a->use_acquire;
    a->consumer_started = 1;
    dsb_ish();
    for (;;) {
        u64 s1 = a->seq;
        if (s1 != last) {
            u64 tmp[7];
            u32 k;
            if (use_acq) dmb_ish();   /* acquire: seq-read before payload read */
            for (k = 0; k < 7U; k++) tmp[k] = a->payload[k];
            if (use_acq) dmb_ish();
            if (a->seq == s1) {       /* stable snapshot */
                a->checks++;
                for (k = 0; k < 7U; k++) {
                    if (tmp[k] != cohdiag_pat(s1, k)) { a->mismatch++; break; }
                }
                last = s1;
            } else {
                a->tears++;           /* seq moved mid-read; retry */
            }
        }
        if (a->producer_done && a->seq == last)
            break;
        if ((++spins & 0xFFFFULL) == 0) watchdog_touch(core_id());
        if (spins > 400000000ULL) break;  /* hard safety cap */
    }
    dsb_ish();
    a->consumer_done = 1;
    dsb_ish();
}

/* Scheduler-loop hook: runs the consumer body on the armed consumer core only. */
void proc_cohdiag_consumer_tick(void)
{
    u32 arm = cohdiag_consumer_arm;
    volatile struct cohdiag_a *a;
    if (arm == 0)
        return;
    if ((arm - 1U) != core_id())
        return;
    a = cohdiag_target;
    if (a)
        cohdiag_consumer_run(a);
    cohdiag_consumer_arm = 0;
    dsb_ish();
}

/* Producer/orchestrator for one publish/observe pass against struct *a. */
static void cohdiag_run_one(volatile struct cohdiag_a *a, u32 use_acquire, u32 iters,
                            u32 consumer_core, u32 *checks, u32 *mismatch,
                            u32 *tears, u32 *timeout)
{
    u32 k, s;
    u64 spins;

    a->seq = 0;
    for (k = 0; k < 7U; k++) a->payload[k] = 0;
    a->producer_done = 0;
    a->consumer_started = 0;
    a->consumer_done = 0;
    a->checks = 0;
    a->mismatch = 0;
    a->tears = 0;
    a->use_acquire = use_acquire;
    dsb_ish();

    cohdiag_target = a;
    dsb_ish();
    cohdiag_consumer_arm = consumer_core + 1U;
    dsb_ish();
    sev();                            /* wake target core from WFE */

    spins = 0;
    while (!a->consumer_started) {
        sev();
        if ((++spins & 0xFFFFULL) == 0) watchdog_touch(core_id());
        if (spins > 200000000ULL) {
            *timeout = 1;
            cohdiag_consumer_arm = 0;
            dsb_ish();
            *checks = a->checks; *mismatch = a->mismatch; *tears = a->tears;
            return;
        }
    }

    for (s = 1; s <= iters; s++) {
        for (k = 0; k < 7U; k++) a->payload[k] = cohdiag_pat(s, k);
        dmb_ish();                    /* release: payload stores before seq store */
        a->seq = s;
        dsb_ish();
        sev();
        for (volatile u32 d = 0; d < 32U; d++) { }  /* light pacing for coverage */
        if ((s & 0x3FFU) == 0) watchdog_touch(core_id());
    }
    a->producer_done = 1;
    dsb_ish();
    sev();

    spins = 0;
    while (!a->consumer_done) {
        sev();
        if ((++spins & 0xFFFFULL) == 0) watchdog_touch(core_id());
        if (spins > 200000000ULL) { *timeout = 1; break; }
    }
    cohdiag_consumer_arm = 0;
    dsb_ish();
    *checks = a->checks;
    *mismatch = a->mismatch;
    *tears = a->tears;
}

/* Write-cost microbench: stride across cache lines (or prime-scatter) on *buf. */
static u64 cohdiag_write_cost_ps(volatile u8 *buf, u32 nlines, u32 count, bool scatter)
{
    u64 t0, t1, dt;
    u32 i, idx = 0;
    for (i = 0; i < nlines; i++) buf[(u64)i * 64ULL] = (u8)i;   /* warm/populate */
    dsb_ish();
    t0 = proc_sched_counter_ticks();
    if (scatter) {
        for (i = 0; i < count; i++) {
            idx += 53U;               /* prime stride */
            if (idx >= nlines) idx -= nlines;
            buf[(u64)idx * 64ULL] = (u8)i;
        }
    } else {
        for (i = 0; i < count; i++) {
            buf[(u64)idx * 64ULL] = (u8)i;
            if (++idx >= nlines) idx = 0;
        }
    }
    t1 = proc_sched_counter_ticks();
    dt = t1 >= t0 ? t1 - t0 : 0;
    return count ? (dt * 18518ULL) / count : 0;
}

/* Same-line repeated write (stays hot in L1 on a cacheable buffer). */
static u64 cohdiag_write_hot_ps(volatile u8 *buf, u32 count)
{
    u64 t0, t1, dt;
    u32 i;
    buf[0] = 1;
    dsb_ish();
    t0 = proc_sched_counter_ticks();
    for (i = 0; i < count; i++) buf[0] = (u8)i;
    t1 = proc_sched_counter_ticks();
    dt = t1 >= t0 ? t1 - t0 : 0;
    return count ? (dt * 18518ULL) / count : 0;
}

bool proc_cohdiag(u32 iters, u32 consumer_core, struct proc_cohdiag_result *out)
{
    struct core_env *e0;
    u64 hp;

    if (!out)
        return false;
    if (core_id() != CORE_NET)        /* producer must own core 0 */
        return false;
    if (iters == 0) iters = 2000;
    if (iters > 20000U) iters = 20000U;
    if (consumer_core != CORE_USERM && consumer_core != CORE_USER0 &&
        consumer_core != CORE_USER1)
        consumer_core = CORE_USER1;   /* default: idle user core 3 */

    simd_zero(out, sizeof(*out));
    out->consumer_core = consumer_core;
    out->producer_seqs = iters;

    /* Part B - effective memory attributes of the live arenas. */
    cohdiag_probe_attr(SHARED_FIFO_BASE, &out->attr_fifo, &out->sh_fifo, &out->par_fault);
    cohdiag_probe_attr(DMA_NET_BASE, &out->attr_dma_net, &out->sh_dma_net, &out->par_fault);
    cohdiag_probe_attr(IPC_SHM_BASE, &out->attr_ipc, &out->sh_ipc, &out->par_fault);
    cohdiag_probe_attr(COHDIAG_WB_SCRATCH, &out->attr_wb, &out->sh_wb, &out->par_fault);
    cohdiag_probe_attr((u64)(usize)cohdiag_nc_buf, &out->attr_nc, &out->sh_nc, &out->par_fault);
    cohdiag_probe_attr((u64)(usize)&__text_start, &out->attr_code, &out->sh_code, &out->par_fault);

    /* Only touch the WB scratch if it really is Normal-WB and clears core0's heap. */
    e0 = core_env_of(CORE_NET);
    hp = (u64)(usize)e0->heap_ptr;
    out->wb_safe = (out->attr_wb == 0xFFU &&
                    COHDIAG_WB_SCRATCH > hp + 0x10000ULL) ? 1U : 0U;

    /* Part C - write cost. NC scratch always; WB scratch only when safe. */
    out->nc_seq_ps     = cohdiag_write_cost_ps(cohdiag_nc_buf, 128U, 4096U, false);
    out->nc_scatter_ps = cohdiag_write_cost_ps(cohdiag_nc_buf, 128U, 4096U, true);
    if (out->wb_safe) {
        volatile u8 *wbuf = (volatile u8 *)(usize)(COHDIAG_WB_SCRATCH + 0x1000UL);
        out->wb_seq_ps     = cohdiag_write_cost_ps(wbuf, 128U, 4096U, false);
        out->wb_scatter_ps = cohdiag_write_cost_ps(wbuf, 128U, 4096U, true);
        out->wb_hot_ps     = cohdiag_write_hot_ps(wbuf, 4096U);
    }

    /* Part A - cross-core publish/observe. */
    if (out->wb_safe) {
        volatile struct cohdiag_a *wb = (volatile struct cohdiag_a *)(usize)COHDIAG_WB_SCRATCH;
        cohdiag_run_one(wb, 1U, iters, consumer_core,
                        &out->wb_checks, &out->wb_mismatch,
                        &out->wb_tears, &out->wb_timeout);
        cohdiag_run_one(wb, 0U, iters, consumer_core,
                        &out->wb_noacq_checks, &out->wb_noacq_mismatch,
                        &out->wb_noacq_tears, &out->wb_noacq_timeout);
    }
    cohdiag_run_one(&cohdiag_nc_a, 1U, iters, consumer_core,
                    &out->nc_checks, &out->nc_mismatch,
                    &out->nc_tears, &out->nc_timeout);

    return true;
}

bool proc_ipc_bench(u32 iterations, struct proc_ipc_bench_result *out)
{
    static u64 bench_payload ALIGNED(64);
    static u8 copy_src[2048] ALIGNED(64);
    static u8 copy_dst[2048] ALIGNED(64);
    static const char name[] = "bench.span";
    static const char copy_name[] = "bench.copy";
    struct proc_ipc_span_desc desc;
    struct proc_ipc_span_desc recv_desc;
    struct irq_frame frame;
    u32 len = 0;
    i32 h;
    i32 hc;
    u32 errors = 0;
    u64 t0, t1;

    if (!out)
        return false;
    if (iterations == 0)
        iterations = 1;
    if (iterations > 100000U)
        iterations = 100000U;

    h = ipc_proc_fifo_create(PRINCIPAL_ROOT, 1, name, PROC_IPC_PEER_ANY,
                             PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV,
                             PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV,
                             PROC_IPC_FIFO_DEPTH_MAX,
                             sizeof(struct proc_ipc_span_desc));
    if (h == PROC_IPC_ERR_EXISTS)
        h = ipc_proc_fifo_open(PRINCIPAL_ROOT, 1, name,
                               PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV);
    if (h < 0) {
        out->iterations = iterations;
        out->desc_size = sizeof(struct proc_ipc_span_desc);
        out->fifo_handle = h;
        out->errors = 1;
        out->svc_ticks = 0;
        out->span_ticks = 0;
        out->span_fast_ticks = 0;
        out->copy64_ticks = 0;
        out->copy512_ticks = 0;
        out->memcpy2048_ticks = 0;
        out->span2048_ticks = 0;
        out->cross_fifo_ticks = 0;
        out->cross_batch_ticks = 0;
        out->cross_ring_batch_ticks = 0;
        out->cross_span_ring_ticks = 0;
        out->cross_span_all_ticks = 0;
        out->span_rt_base_ticks = 0;
        out->span_rt_ish_ticks = 0;
        out->span_rt_acqrel_ticks = 0;
        out->span_rt_asm_ticks = 0;
        out->sev_ticks = 0;
        return false;
    }

    hc = ipc_proc_fifo_create(PRINCIPAL_ROOT, 1, copy_name, PROC_IPC_PEER_ANY,
                              PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV,
                              PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV,
                              PROC_IPC_FIFO_DEPTH_MAX, 512);
    if (hc == PROC_IPC_ERR_EXISTS)
        hc = ipc_proc_fifo_open(PRINCIPAL_ROOT, 1, copy_name,
                                PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV);
    if (hc < 0)
        errors++;

    while (ipc_proc_fifo_recv(PRINCIPAL_ROOT, h, &recv_desc, sizeof(recv_desc), &len) == PROC_IPC_OK)
        ;

    simd_zero(&frame, sizeof(frame));
    t0 = proc_sched_counter_ticks();
    for (u32 i = 0; i < iterations; i++) {
        if (!proc_handle_svc_inner(&frame, ((u64)EC_SVC64 << ESR_EC_SHIFT) | PROC_SVC_GETPID, false))
            errors++;
    }
    t1 = proc_sched_counter_ticks();
    out->svc_ticks = t1 >= t0 ? t1 - t0 : 0;

    bench_payload = 0x50494F5300000000ULL | iterations;
    desc.addr = (u64)(usize)&bench_payload;
    desc.len = sizeof(bench_payload);
    desc.flags = PROC_IPC_SPAN_F_READONLY;
    desc.tag = 0xBEE00000ULL;

    t0 = proc_sched_counter_ticks();
    for (u32 i = 0; i < iterations; i++) {
        desc.tag = 0xBEE00000ULL | i;
        if (ipc_proc_fifo_send(PRINCIPAL_ROOT, h, &desc, sizeof(desc)) != PROC_IPC_OK ||
            ipc_proc_fifo_recv(PRINCIPAL_ROOT, h, &recv_desc, sizeof(recv_desc), &len) != PROC_IPC_OK ||
            len != sizeof(recv_desc) || recv_desc.tag != desc.tag)
            errors++;
    }
    t1 = proc_sched_counter_ticks();
    out->span_ticks = t1 >= t0 ? t1 - t0 : 0;

    while (ipc_proc_fifo_recv_span(PRINCIPAL_ROOT, h, &recv_desc) == PROC_IPC_OK)
        ;

    t0 = proc_sched_counter_ticks();
    for (u32 i = 0; i < iterations; i++) {
        desc.tag = 0xFA570000ULL | i;
        if (ipc_proc_fifo_send_span(PRINCIPAL_ROOT, h, &desc) != PROC_IPC_OK ||
            ipc_proc_fifo_recv_span(PRINCIPAL_ROOT, h, &recv_desc) != PROC_IPC_OK ||
            recv_desc.tag != desc.tag)
            errors++;
    }
    t1 = proc_sched_counter_ticks();
    out->span_fast_ticks = t1 >= t0 ? t1 - t0 : 0;

    for (u32 i = 0; i < sizeof(copy_src); i++) {
        copy_src[i] = (u8)(i ^ (i >> 3) ^ 0x5AU);
        copy_dst[i] = 0;
    }

    if (hc >= 0) {
        while (ipc_proc_fifo_recv(PRINCIPAL_ROOT, hc, copy_dst, sizeof(copy_dst), &len) == PROC_IPC_OK)
            ;
        t0 = proc_sched_counter_ticks();
        for (u32 i = 0; i < iterations; i++) {
            if (ipc_proc_fifo_send(PRINCIPAL_ROOT, hc, copy_src, 64) != PROC_IPC_OK ||
                ipc_proc_fifo_recv(PRINCIPAL_ROOT, hc, copy_dst, 64, &len) != PROC_IPC_OK ||
                len != 64)
                errors++;
        }
        t1 = proc_sched_counter_ticks();
        out->copy64_ticks = t1 >= t0 ? t1 - t0 : 0;

        t0 = proc_sched_counter_ticks();
        for (u32 i = 0; i < iterations; i++) {
            if (ipc_proc_fifo_send(PRINCIPAL_ROOT, hc, copy_src, 512) != PROC_IPC_OK ||
                ipc_proc_fifo_recv(PRINCIPAL_ROOT, hc, copy_dst, 512, &len) != PROC_IPC_OK ||
                len != 512)
                errors++;
        }
        t1 = proc_sched_counter_ticks();
        out->copy512_ticks = t1 >= t0 ? t1 - t0 : 0;
    }

    t0 = proc_sched_counter_ticks();
    for (u32 i = 0; i < iterations; i++)
        simd_memcpy(copy_dst, copy_src, 2048);
    t1 = proc_sched_counter_ticks();
    out->memcpy2048_ticks = t1 >= t0 ? t1 - t0 : 0;

    desc.addr = (u64)(usize)copy_src;
    desc.len = 2048;
    desc.flags = PROC_IPC_SPAN_F_READONLY;
    t0 = proc_sched_counter_ticks();
    for (u32 i = 0; i < iterations; i++) {
        desc.tag = 0xF2048000ULL | i;
        if (ipc_proc_fifo_send_span(PRINCIPAL_ROOT, h, &desc) != PROC_IPC_OK ||
            ipc_proc_fifo_recv_span(PRINCIPAL_ROOT, h, &recv_desc) != PROC_IPC_OK ||
            recv_desc.tag != desc.tag || recv_desc.len != 2048)
            errors++;
    }
    t1 = proc_sched_counter_ticks();
    out->span2048_ticks = t1 >= t0 ? t1 - t0 : 0;

    if (core_id() == CORE_NET) {
        struct fifo_msg m;
        struct fifo_msg r;
        while (fifo_pop(CORE_NET, CORE_USERM, &r))
            ;
        t0 = proc_sched_counter_ticks();
        for (u32 i = 0; i < iterations; i++) {
            m.type = MSG_BENCH_ECHO;
            m.param = sizeof(desc);
            m.buffer = (u64)(usize)&bench_payload;
            m.length = sizeof(bench_payload);
            m.status = 0;
            m.tag = 0xC0550000ULL | i;
            m.timestamp = t0;
            m._reserved = 0;
            if (!fifo_push(CORE_NET, CORE_USERM, &m)) {
                errors++;
                continue;
            }
            u32 spins = 1000000U;
            while (!fifo_pop(CORE_NET, CORE_USERM, &r) && spins--)
                ;
            if (spins == 0 || r.type != MSG_ACK || r.tag != m.tag)
                errors++;
        }
        t1 = proc_sched_counter_ticks();
        out->cross_fifo_ticks = t1 >= t0 ? t1 - t0 : 0;

        while (fifo_pop(CORE_NET, CORE_USERM, &r))
            ;
        const u32 batch = 16U;
        u32 sent = 0;
        t0 = proc_sched_counter_ticks();
        while (sent < iterations) {
            u32 n = iterations - sent;
            if (n > batch) n = batch;
            for (u32 i = 0; i < n; i++) {
                m.type = MSG_BENCH_BATCH;
                m.param = sizeof(desc);
                m.buffer = (u64)(usize)&bench_payload;
                m.length = n;
                m.status = 0;
                m.tag = 0xBA7C0000ULL | (sent + i);
                m.timestamp = t0;
                m._reserved = 0;
                if (!fifo_push(CORE_NET, CORE_USERM, &m))
                    errors++;
            }
            u32 spins = 1000000U;
            while (!fifo_pop(CORE_NET, CORE_USERM, &r) && spins--)
                ;
            if (spins == 0 || r.type != MSG_ACK)
                errors++;
            sent += n;
        }
        t1 = proc_sched_counter_ticks();
        out->cross_batch_ticks = t1 >= t0 ? t1 - t0 : 0;

        while (fifo_pop(CORE_NET, CORE_USERM, &r))
            ;
        const u32 micro_full = 16U;
        const u32 micro_partial = 4U;
        sent = 0;
        t0 = proc_sched_counter_ticks();
        while (sent < iterations) {
            u32 n = iterations - sent;
            if (n > micro_full) n = micro_full;
            for (u32 i = 0; i < n; i++) {
                m.type = MSG_BENCH_BATCH;
                m.param = sizeof(desc);
                m.buffer = (u64)(usize)&bench_payload;
                m.length = n;
                m.status = 0;
                m.tag = 0xF0110000ULL | (sent + i);
                m.timestamp = t0;
                m._reserved = 0;
                if (!fifo_push(CORE_NET, CORE_USERM, &m))
                    errors++;
            }
            u32 spins = 1000000U;
            while (!fifo_pop(CORE_NET, CORE_USERM, &r) && spins--)
                ;
            if (spins == 0 || r.type != MSG_ACK)
                errors++;
            sent += n;
        }
        t1 = proc_sched_counter_ticks();
        out->cross_micro_full_ticks = t1 >= t0 ? t1 - t0 : 0;

        while (fifo_pop(CORE_NET, CORE_USERM, &r))
            ;
        sent = 0;
        t0 = proc_sched_counter_ticks();
        while (sent < iterations) {
            u32 n = iterations - sent;
            if (n > micro_partial) n = micro_partial;
            for (u32 i = 0; i < n; i++) {
                m.type = MSG_BENCH_BATCH;
                m.param = sizeof(desc);
                m.buffer = (u64)(usize)&bench_payload;
                m.length = n;
                m.status = 0;
                m.tag = 0xF0040000ULL | (sent + i);
                m.timestamp = t0;
                m._reserved = 0;
                if (!fifo_push(CORE_NET, CORE_USERM, &m))
                    errors++;
            }
            u32 spins = 1000000U;
            while (!fifo_pop(CORE_NET, CORE_USERM, &r) && spins--)
                ;
            if (spins == 0 || r.type != MSG_ACK)
                errors++;
            sent += n;
        }
        t1 = proc_sched_counter_ticks();
        out->cross_micro_partial_ticks = t1 >= t0 ? t1 - t0 : 0;

        while (fifo_pop(CORE_NET, CORE_USERM, &r))
            ;
        struct fifo_msg mb[16];
        sent = 0;
        t0 = proc_sched_counter_ticks();
        while (sent < iterations) {
            u32 n = iterations - sent;
            if (n > micro_full) n = micro_full;
            for (u32 i = 0; i < n; i++) {
                mb[i].type = MSG_BENCH_BATCH;
                mb[i].param = sizeof(desc);
                mb[i].buffer = (u64)(usize)&bench_payload;
                mb[i].length = n;
                mb[i].status = 0;
                mb[i].tag = 0xF1F00000ULL | (sent + i);
                mb[i].timestamp = t0;
                mb[i]._reserved = 0;
            }
            if (fifo_push_batch(CORE_NET, CORE_USERM, mb, n) != n)
                errors++;
            u32 spins = 1000000U;
            while (!fifo_pop(CORE_NET, CORE_USERM, &r) && spins--)
                ;
            if (spins == 0 || r.type != MSG_ACK)
                errors++;
            sent += n;
        }
        t1 = proc_sched_counter_ticks();
        out->cross_ring_batch_ticks = t1 >= t0 ? t1 - t0 : 0;

        struct fifo_span_msg sb[16];
        while (fifo_pop(CORE_NET, CORE_USERM, &r))
            ;
        while (fifo_span_pop_batch(CORE_NET, CORE_USERM, sb, 16))
            ;
        sent = 0;
        t0 = proc_sched_counter_ticks();
        while (sent < iterations) {
            u32 n = iterations - sent;
            if (n > micro_full) n = micro_full;
            for (u32 i = 0; i < n; i++) {
                sb[i].addr = (u64)(usize)copy_src;
                sb[i].tag = 0x5A5A0000ULL | (sent + i);
                sb[i].len = 2048;
                sb[i].flags = PROC_IPC_SPAN_F_READONLY;
                sb[i].aux = 0;
                sb[i]._pad = 0;
            }
            if (fifo_span_push_batch(CORE_NET, CORE_USERM, sb, n) != n)
                errors++;
            u32 spins = 1000000U;
            u32 got = 0;
            while (got == 0 && spins--) {
                got = fifo_span_pop_batch(CORE_NET, CORE_USERM, sb, 16);
                ;
            }
            if (spins == 0 || got == 0 || got > n)
                errors++;
            sent += n;
        }
        t1 = proc_sched_counter_ticks();
        out->cross_span_ring_ticks = t1 >= t0 ? t1 - t0 : 0;

        const u32 targets[3] = { CORE_USERM, CORE_USER0, CORE_USER1 };
        for (u32 ti = 0; ti < 3; ti++) {
            while (fifo_pop(CORE_NET, targets[ti], &r))
                ;
            while (fifo_span_pop_batch(CORE_NET, targets[ti], sb, 16))
                ;
        }
        sent = 0;
        t0 = proc_sched_counter_ticks();
        while (sent < iterations) {
            u32 pending[3] = {0, 0, 0};
            for (u32 ti = 0; ti < 3 && sent < iterations; ti++) {
                u32 n = iterations - sent;
                if (n > micro_full) n = micro_full;
                for (u32 i = 0; i < n; i++) {
                    sb[i].addr = (u64)(usize)copy_src;
                    sb[i].tag = 0xA1100000ULL | (sent + i);
                    sb[i].len = 2048;
                    sb[i].flags = PROC_IPC_SPAN_F_READONLY;
                    sb[i].aux = ti;
                    sb[i]._pad = 0;
                }
                if (fifo_span_push_batch(CORE_NET, targets[ti], sb, n) != n)
                    errors++;
                pending[ti] = n;
                sent += n;
            }
            for (u32 ti = 0; ti < 3; ti++) {
                if (!pending[ti])
                    continue;
                u32 spins = 1000000U;
                u32 got = 0;
                while (got == 0 && spins--)
                    got = fifo_span_pop_batch(CORE_NET, targets[ti], sb, 16);
                if (spins == 0 || got == 0 || got > pending[ti])
                    errors++;
            }
        }
        t1 = proc_sched_counter_ticks();
        out->cross_span_all_ticks = t1 >= t0 ? t1 - t0 : 0;

        /* ── A/B: span-ring descriptor copy + memory-ordering strategy ──
         * Same-core round-trip on the unused (CORE_NET,CORE_NET) self-ring
         * isolates raw copy + barrier cost from scheduler/wakeup noise:
         *   base   = DMB-SY + compiler 32B struct copy   (current production)
         *   acqrel = LDAR/STLR + DSB ISHST, compiler copy
         *   asm    = same ordering, hand ldp/stp q (NEON) 32B copy
         * Each iteration pushes then pops one chunk; per-desc = ticks/iters. */
        {
            struct fifo_span_msg rb[16];
            const u32 ab_chunk = 16U;
            for (u32 i = 0; i < ab_chunk; i++) {
                sb[i].addr = (u64)(usize)copy_src;
                sb[i].tag = 0x5C5C0000ULL | i;
                sb[i].len = 2048;
                sb[i].flags = PROC_IPC_SPAN_F_READONLY;
                sb[i].aux = 0;
                sb[i]._pad = 0;
            }

            while (fifo_span_pop_batch(CORE_NET, CORE_NET, rb, 16))
                ;
            t0 = proc_sched_counter_ticks();
            for (u32 done = 0; done < iterations; done += ab_chunk) {
                if (fifo_span_push_batch(CORE_NET, CORE_NET, sb, ab_chunk) != ab_chunk)
                    errors++;
                if (fifo_span_pop_batch(CORE_NET, CORE_NET, rb, ab_chunk) != ab_chunk)
                    errors++;
            }
            t1 = proc_sched_counter_ticks();
            out->span_rt_base_ticks = t1 >= t0 ? t1 - t0 : 0;

            while (fifo_span_pop_batch(CORE_NET, CORE_NET, rb, 16))
                ;
            t0 = proc_sched_counter_ticks();
            for (u32 done = 0; done < iterations; done += ab_chunk) {
                if (fifo_span_push_batch_ish(CORE_NET, CORE_NET, sb, ab_chunk) != ab_chunk)
                    errors++;
                if (fifo_span_pop_batch_ish(CORE_NET, CORE_NET, rb, ab_chunk) != ab_chunk)
                    errors++;
            }
            t1 = proc_sched_counter_ticks();
            out->span_rt_ish_ticks = t1 >= t0 ? t1 - t0 : 0;

            while (fifo_span_pop_batch(CORE_NET, CORE_NET, rb, 16))
                ;
            t0 = proc_sched_counter_ticks();
            for (u32 done = 0; done < iterations; done += ab_chunk) {
                if (fifo_span_push_batch_acqrel(CORE_NET, CORE_NET, sb, ab_chunk) != ab_chunk)
                    errors++;
                if (fifo_span_pop_batch_acqrel(CORE_NET, CORE_NET, rb, ab_chunk) != ab_chunk)
                    errors++;
            }
            t1 = proc_sched_counter_ticks();
            out->span_rt_acqrel_ticks = t1 >= t0 ? t1 - t0 : 0;

            while (fifo_span_pop_batch(CORE_NET, CORE_NET, rb, 16))
                ;
            t0 = proc_sched_counter_ticks();
            for (u32 done = 0; done < iterations; done += ab_chunk) {
                if (fifo_span_push_batch_asm(CORE_NET, CORE_NET, sb, ab_chunk) != ab_chunk)
                    errors++;
                if (fifo_span_pop_batch_asm(CORE_NET, CORE_NET, rb, ab_chunk) != ab_chunk)
                    errors++;
            }
            t1 = proc_sched_counter_ticks();
            out->span_rt_asm_ticks = t1 >= t0 ? t1 - t0 : 0;
        }
    }

    t0 = proc_sched_counter_ticks();
    for (u32 i = 0; i < iterations; i++)
        sev();
    t1 = proc_sched_counter_ticks();
    out->sev_ticks = t1 >= t0 ? t1 - t0 : 0;

    out->iterations = iterations;
    out->desc_size = sizeof(struct proc_ipc_span_desc);
    out->fifo_handle = h;
    out->errors = errors;
    return errors == 0;
}

u32 proc_sched_snapshot(struct proc_sched_core_snapshot *out, u32 max_entries)
{
    if (!out || max_entries == 0)
        return 0;
    u32 n = max_entries < 3U ? max_entries : 3U;
    u64 now = proc_sched_counter_ticks();
    for (u32 i = 0; i < n; i++) {
        u64 start = sched_start_ticks_core[i];
        u64 total = (start != 0 && now >= start) ? (now - start) : 0;
        u64 idle = sched_idle_ticks_core[i];
        u64 idle_enter = sched_idle_enter_ticks_core[i];
        if (idle_enter != 0 && now >= idle_enter)
            idle += now - idle_enter;
        if (idle > total)
            idle = total;
        u64 busy = total - idle;
        u64 t = total;
        u64 b = busy;
        while (t > 0x00FFFFFFFFFFFFFFULL) {
            t >>= 1;
            b >>= 1;
        }
        out[i].core = i + CORE_USERM;
        out[i].busy_permille = t ? (u32)((b * 1000ULL) / t) : 0;
        out[i].active_count = 0;
        out[i].idle_count = sched_idle_count_core[i];
        out[i].wake_count = sched_wake_count_core[i];
        out[i].idle_ticks = idle;
        out[i].total_ticks = total;
        out[i].preemptions = preempt_count_core[i];
        out[i].soft_events = soft_event_count_core[i];
        out[i].soft_boosts = soft_boost_count_core[i];
    }
    return n;
}

u32 proc_snapshot(struct proc_ui_entry *out, u32 max_entries)
{
    if (!out || max_entries == 0)
        return 0;

    struct process snap[MAX_PROCS_PER_CORE];
    u32 snap_bump[MAX_PROCS_PER_CORE];
    u32 n = 0;
    u64 total_runtime = 0;
    u64 now = timer_ticks();

    simd_zero(&out[0], sizeof(out[0]));
    out[0].pid = PROC_UI_KERNEL_PID;
    out[0].parent_pid = PROC_UI_KERNEL_PARENT_PID;
    out[0].principal_id = PRINCIPAL_ROOT;
    out[0].state = PROC_RUNNING;
    out[0].affinity_core = 0;
    out[0].priority_class = PROC_PRIO_REALTIME;
    out[0].runtime_ticks = now;
    out[0].cpu_percent = 100;
    u64 k_static = proc_kernel_static_bytes();
    u64 k_core = proc_kernel_core_private_bytes();
    u64 k_total = k_static + k_core;
    out[0].mem_kib = (u32)(k_total >> 10);
    out[0].arena_capacity_kib = (u32)((CORE_PRIV_SIZE * 4ULL) >> 10);
    out[0].arena_used_kib = (u32)(k_core >> 10);
    out[0].arena_high_kib = out[0].arena_used_kib;
    out[0].arena_bump_kib = out[0].arena_used_kib;
    out[0].arena_span_kib = (u32)(k_static >> 10);
    out[0].arena_span_count = 4;
    out[0].image_path[0] = '[';
    out[0].image_path[1] = 'k';
    out[0].image_path[2] = 'e';
    out[0].image_path[3] = 'r';
    out[0].image_path[4] = 'n';
    out[0].image_path[5] = 'e';
    out[0].image_path[6] = 'l';
    out[0].image_path[7] = ']';
    out[0].image_path[8] = 0;
    if (max_entries == 1)
        return 1;

    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        struct process p = procs[i];
        if (p.state != PROC_READY && p.state != PROC_RUNNING && p.state != PROC_BLOCKED)
            continue;
        if (p.state == PROC_RUNNING && now >= p.ticks)
            p.runtime_ticks += (now - p.ticks);
        u32 bump = 0;
        if (p.arena_base != 0 && heap_top[i] > p.arena_base) {
            u64 b = heap_top[i] - p.arena_base;
            bump = b > 0xFFFFFFFFULL ? 0xFFFFFFFFU : (u32)b;
        }
        snap[n] = p;
        snap_bump[n] = bump;
        n++;
        total_runtime += p.runtime_ticks;
        if (n == MAX_PROCS_PER_CORE)
            break;
    }

    if (total_runtime == 0)
        total_runtime = 1;

    u32 avail = max_entries - 1U;
    u32 out_n = (n < avail) ? n : avail;
    for (u32 i = 0; i < out_n; i++) {
        u32 oi = i + 1U;
        out[oi].pid = snap[i].pid;
        out[oi].parent_pid = snap[i].parent_pid;
        out[oi].principal_id = snap[i].principal_id;
        out[oi].state = snap[i].state;
        out[oi].affinity_core = snap[i].affinity_core;
        out[oi].priority_class = snap[i].priority_class;
        out[oi].mem_kib = snap[i].mem_size >> 10;
        out[oi].arena_capacity_kib = snap[i].arena_capacity_bytes >> 10;
        u32 bump = snap_bump[i];
        u64 used = (u64)bump + snap[i].arena_span_bytes;
        if (used > 0xFFFFFFFFULL)
            used = 0xFFFFFFFFULL;
        out[oi].arena_used_kib = (u32)used >> 10;
        out[oi].arena_high_kib = snap[i].arena_high_bytes >> 10;
        out[oi].arena_bump_kib = bump >> 10;
        out[oi].arena_span_kib = snap[i].arena_span_bytes >> 10;
        out[oi].arena_span_count = snap[i].arena_span_count;
        out[oi].cpu_percent = (u32)((snap[i].runtime_ticks * 100ULL) / total_runtime);
        out[oi].preemptions = snap[i].preemptions;
        out[oi].runtime_ticks = snap[i].runtime_ticks;
        for (u32 j = 0; j < sizeof(out[oi].image_path); j++)
            out[oi].image_path[j] = snap[i].image_path[j];
        out[oi].image_path[sizeof(out[oi].image_path) - 1] = 0;
    }
    return out_n + 1U;
}

u32 proc_log_snapshot(struct proc_log_ui_entry *out, u32 max_entries)
{
    if (!out || max_entries == 0)
        return 0;
    u32 n = 0;
    for (u32 cs = 0; cs < 3 && n < max_entries; cs++) {
        struct appf_ring_log *q = &appf_logs[cs];
        u32 head = q->head;
        u32 tail = q->tail;
        if (head >= APPF_LOG_RING_SIZE || tail >= APPF_LOG_RING_SIZE)
            continue;
        u32 i = q->tail;
        u32 scanned = 0;
        while (i != head && scanned < APPF_LOG_RING_SIZE && n < max_entries) {
            struct appf_log_record *r = &q->recs[i];
            out[n].core = cs + CORE_USERM;
            out[n].seq = r->seq;
            out[n].level = r->level;
            out[n].len = r->len;
            u32 copy = r->len < APPF_LOG_MSG_MAX ? r->len : APPF_LOG_MSG_MAX;
            for (u32 j = 0; j < copy; j++)
                out[n].msg[j] = (char)r->msg[j];
            out[n].msg[copy] = 0;
            n++;
            i = (i + 1U) % APPF_LOG_RING_SIZE;
            scanned++;
        }
    }
    return n;
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

const char *proc_image_format_name(u32 format)
{
    switch (format) {
    case PROC_IMAGE_FORMAT_FLAT:  return "flat";
    case PROC_IMAGE_FORMAT_PIX:   return "pix";
    case PROC_IMAGE_FORMAT_ELF64: return "elf64";
    default:                      return "none";
    }
}

const char *proc_image_status_name(u32 status)
{
    switch (status) {
    case PROC_IMAGE_STATUS_OK:             return "ok";
    case PROC_IMAGE_STATUS_NOT_FOUND:      return "not_found";
    case PROC_IMAGE_STATUS_STAT_FAILED:    return "stat_failed";
    case PROC_IMAGE_STATUS_DIRECTORY:      return "directory";
    case PROC_IMAGE_STATUS_INVALID_SIZE:   return "invalid_size";
    case PROC_IMAGE_STATUS_READ_FAILED:    return "read_failed";
    case PROC_IMAGE_STATUS_UNSUPPORTED:    return "unsupported";
    case PROC_IMAGE_STATUS_HEADER_INVALID: return "header_invalid";
    case PROC_IMAGE_STATUS_HEADER_PARTIAL: return "header_partial";
    default:                               return "unknown";
    }
}

const char *proc_image_launch_mode_name(u32 mode)
{
    switch (mode) {
    case PROC_IMAGE_LAUNCH_FLAT_DIRECT:   return "flat-compatible";
    case PROC_IMAGE_LAUNCH_LOADER_NEEDED: return "blocked-loader-required";
    default:                              return "blocked";
    }
}

static u32 proc_image_magic32(const u8 *p)
{
    return ((u32)p[0]) | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void proc_image_copy_info(struct proc_image_validation *out,
                                 const struct pix_image_info *info)
{
    out->image_type = info->type;
    out->image_flags = info->flags;
    out->validator_status = info->status;
    out->entry_offset = info->entry_offset;
    out->code_size = info->code_size;
    out->data_size = info->data_size;
    out->bss_size = info->bss_size;
    out->load_span = info->load_span;
    out->stack_size = info->stack_size;
    out->min_memory = info->min_memory;
    out->reloc_count = info->reloc_count;
    out->import_count = info->import_count;
}

bool proc_validate_image_path(const char *path, struct proc_image_validation *out)
{
    if (!out)
        return false;
    simd_zero(out, sizeof(*out));
    out->status = PROC_IMAGE_STATUS_NOT_FOUND;
    out->format = PROC_IMAGE_FORMAT_NONE;
    out->launch_mode = PROC_IMAGE_LAUNCH_BLOCKED;

    if (!path || !path[0])
        return false;

    u64 inode = walfs_find(path);
    if (!inode)
        return false;

    struct walfs_inode info;
    if (!walfs_stat(inode, &info)) {
        out->status = PROC_IMAGE_STATUS_STAT_FAILED;
        return false;
    }
    if (info.flags & WALFS_DIR) {
        out->status = PROC_IMAGE_STATUS_DIRECTORY;
        return false;
    }
    if (info.size == 0 || info.size > PROC_SLOT_SIZE - 64U) {
        out->status = PROC_IMAGE_STATUS_INVALID_SIZE;
        out->file_bytes = (u32)info.size;
        return false;
    }

    out->file_bytes = (u32)info.size;
    out->header_bytes = out->file_bytes < PROC_IMAGE_VALIDATE_HEADER_MAX ?
                        out->file_bytes : PROC_IMAGE_VALIDATE_HEADER_MAX;
    u32 loaded = walfs_read(inode, 0, proc_image_validate_scratch, out->header_bytes);
    if (loaded != out->header_bytes) {
        out->status = PROC_IMAGE_STATUS_READ_FAILED;
        out->header_bytes = loaded;
        return false;
    }

    if (out->header_bytes >= 4 && proc_image_magic32(proc_image_validate_scratch) == PIX_MAGIC) {
        struct pix_image_info pi;
        out->format = PROC_IMAGE_FORMAT_PIX;
        bool ok = pix_validate_header(proc_image_validate_scratch, out->header_bytes,
                                      out->file_bytes, PROC_SLOT_SIZE, &pi);
        proc_image_copy_info(out, &pi);
        out->launch_mode = PROC_IMAGE_LAUNCH_LOADER_NEEDED;
        if (!ok) {
            out->status = (pi.status == PIX_VALIDATE_SHORT_HEADER ||
                           pi.status == PIX_VALIDATE_PARTIAL) ?
                          PROC_IMAGE_STATUS_HEADER_PARTIAL :
                          PROC_IMAGE_STATUS_HEADER_INVALID;
            return false;
        }
        if (pi.type != PIX_EXEC) {
            out->status = PROC_IMAGE_STATUS_UNSUPPORTED;
            return false;
        }
        out->status = PROC_IMAGE_STATUS_OK;
        return true;
    }

    if (out->header_bytes >= 4 &&
        proc_image_validate_scratch[0] == 0x7F &&
        proc_image_validate_scratch[1] == 'E' &&
        proc_image_validate_scratch[2] == 'L' &&
        proc_image_validate_scratch[3] == 'F') {
        struct pix_image_info ei;
        out->format = PROC_IMAGE_FORMAT_ELF64;
        bool ok = elf64_validate_header(proc_image_validate_scratch, out->header_bytes,
                                        out->file_bytes, PROC_SLOT_SIZE, &ei);
        proc_image_copy_info(out, &ei);
        out->launch_mode = PROC_IMAGE_LAUNCH_LOADER_NEEDED;
        if (!ok) {
            out->status = (ei.status == PIX_VALIDATE_PARTIAL) ?
                          PROC_IMAGE_STATUS_HEADER_PARTIAL :
                          PROC_IMAGE_STATUS_HEADER_INVALID;
            return false;
        }
        out->status = PROC_IMAGE_STATUS_OK;
        return true;
    }

    out->status = PROC_IMAGE_STATUS_OK;
    out->format = PROC_IMAGE_FORMAT_FLAT;
    out->launch_mode = PROC_IMAGE_LAUNCH_FLAT_DIRECT;
    out->entry_offset = 0;
    out->code_size = out->file_bytes;
    out->load_span = out->file_bytes;
    return true;
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

i32 proc_restart_pid(u32 pid, u32 code)
{
    char path[PROC_LAUNCH_PATH_MAX];
    u32 target_core = 0;
    u32 principal_id = PRINCIPAL_ROOT;
    u32 priority_class = PROC_PRIO_NORMAL;

    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        if (procs[i].pid != pid)
            continue;
        if (procs[i].state != PROC_READY && procs[i].state != PROC_RUNNING &&
            procs[i].state != PROC_BLOCKED)
            return -1;
        if (procs[i].image_path[0] == 0)
            return -1;

        u32 j = 0;
        for (; j + 1 < sizeof(path) && procs[i].image_path[j]; j++)
            path[j] = procs[i].image_path[j];
        path[j] = 0;
        target_core = procs[i].affinity_core;
        principal_id = procs[i].principal_id;
        priority_class = procs[i].priority_class;

        if (procs[i].state == PROC_RUNNING)
            proc_account_runtime(&procs[i]);
        procs[i].state = PROC_DEAD;
        procs[i].exit_code = code;
        return proc_launch_on_core_as_prio(target_core, path, principal_id, priority_class);
    }
    return -1;
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
            if (procs[i].arena_span_count != 0)
                return false;
            u64 hb = heap_top[i];
            u64 lo = procs[i].arena_base ? procs[i].arena_base : (u64)(usize)procs[i].base;
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
            launch_req[uc].migrate_arena_high_bytes = procs[i].arena_high_bytes;
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

static i32 sys_park(void)
{
    proc_park();
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
    u64 arena_base = p->arena_base ? p->arena_base : (u64)(usize)p->base;
    u64 new_top;
    if (increment >= 0) {
        new_top = old + (u32)increment;
        if (new_top < old)
            return (void *)(usize)-1;
    } else {
        u32 dec = (u32)(-increment);
        if (old < arena_base + dec)
            return (void *)(usize)-1;
        new_top = old - dec;
    }
    u64 limit = p->arena_limit ? p->arena_limit : (u64)(usize)p->base + p->mem_size - 65536;
    u64 span_floor = proc_span_floor[current_proc] ? proc_span_floor[current_proc] : limit;
    if (span_floor < limit)
        limit = span_floor;
    if (p->capsule_enabled && p->quota_mem_kib > 0) {
        u64 qlim = (u64)(usize)p->base + ((u64)p->quota_mem_kib << 10);
        if (qlim < limit) limit = qlim;
    }
    if (new_top > limit || new_top < arena_base)
        return (void *)(usize)-1;
    heap_top[current_proc] = new_top;
    proc_arena_update_high(p, current_proc);
    return (void *)(usize)old;
}

void *proc_span_rent(u32 bytes, u32 align, u32 type)
{
    if (!on_user_core() || bytes == 0 || type >= MEM_ARENA_TYPE_COUNT)
        return NULL;
    if (align < 16)
        align = 16;
    if ((align & (align - 1U)) != 0 || align > 4096U)
        return NULL;
    struct process *p = &procs[current_proc];
    if (p->state != PROC_RUNNING && p->state != PROC_READY && p->state != PROC_BLOCKED)
        return NULL;
    i32 free_slot = -1;
    for (u32 i = 0; i < PROC_SPANS_PER_PROCESS; i++) {
        if (!proc_spans[current_proc][i].used) {
            free_slot = (i32)i;
            break;
        }
    }
    if (free_slot < 0)
        return NULL;
    if (bytes > p->arena_capacity_bytes)
        return NULL;
    u64 floor = proc_span_floor[current_proc] ? proc_span_floor[current_proc] : p->arena_limit;
    if ((u64)bytes > floor)
        return NULL;
    u64 candidate = (floor - bytes) & ~((u64)align - 1ULL);
    if (candidate + bytes > floor || candidate < heap_top[current_proc] || candidate < p->arena_base)
        return NULL;
    struct proc_span_slot *s = &proc_spans[current_proc][(u32)free_slot];
    s->used = true;
    s->type = type;
    s->size = bytes;
    s->addr = candidate;
    proc_span_floor[current_proc] = candidate;
    p->arena_span_bytes += bytes;
    p->arena_span_count++;
    proc_arena_update_high(p, current_proc);
    return (void *)(usize)candidate;
}

bool proc_span_release(void *ptr)
{
    if (!on_user_core() || !ptr)
        return false;
    u64 addr = (u64)(usize)ptr;
    struct process *p = &procs[current_proc];
    bool released = false;
    for (u32 i = 0; i < PROC_SPANS_PER_PROCESS; i++) {
        struct proc_span_slot *s = &proc_spans[current_proc][i];
        if (!s->used || s->addr != addr)
            continue;
        if (p->arena_span_bytes >= s->size)
            p->arena_span_bytes -= s->size;
        else
            p->arena_span_bytes = 0;
        if (p->arena_span_count > 0)
            p->arena_span_count--;
        s->used = false;
        released = true;
        break;
    }
    if (!released)
        return false;
    u64 floor = p->arena_limit;
    for (u32 i = 0; i < PROC_SPANS_PER_PROCESS; i++) {
        struct proc_span_slot *s = &proc_spans[current_proc][i];
        if (s->used && s->addr < floor)
            floor = s->addr;
    }
    proc_span_floor[current_proc] = floor;
    proc_arena_update_high(p, current_proc);
    return true;
}

static void *sys_span_rent(u32 bytes, u32 align, u32 type)
{
    return proc_span_rent(bytes, align, type);
}

static i32 sys_span_release(void *ptr)
{
    return proc_span_release(ptr) ? 0 : -1;
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
    i32 r = ipc_proc_fifo_send(principal_current(), channel_id, data, len);
    if (r == PROC_IPC_OK)
        (void)proc_soft_event(ipc_proc_fifo_owner_pid(channel_id), PROC_SOFT_EVENT_IPC_FIFO, true);
    return r;
}

static i32 sys_ipc_fifo_send_span(i32 channel_id, const void *addr, u32 len, u32 flags, u64 tag)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    if ((flags & ~(PROC_IPC_SPAN_F_SHARED | PROC_IPC_SPAN_F_READONLY | PROC_IPC_SPAN_F_DMA)) != 0)
        return PROC_IPC_ERR_INVAL;
    if (!ipc_proc_fifo_is_span_desc(channel_id)) return PROC_IPC_ERR_INVAL;
    if (!ptr_valid(addr, len)) return PROC_IPC_ERR_INVAL;
    struct proc_ipc_span_desc d = {
        .addr = (u64)(usize)addr,
        .len = len,
        .flags = flags,
        .tag = tag,
    };
    i32 r = ipc_proc_fifo_send_span(principal_current(), channel_id, &d);
    if (r == PROC_IPC_OK)
        (void)proc_soft_event(ipc_proc_fifo_owner_pid(channel_id), PROC_SOFT_EVENT_IPC_FIFO, true);
    return r;
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

static i32 sys_sw_int_kernel(i32 channel_id, u32 event_type, u32 flags)
{
    if (!has_ipc_cap()) return PROC_IPC_ERR_ACCESS;
    if ((flags & ~PROC_SW_INT_F_BOOST) != 0) return PROC_IPC_ERR_INVAL;
    u32 target_pid = ipc_proc_fifo_owner_pid(channel_id);
    if (target_pid == 0) return PROC_IPC_ERR_NOENT;
    if (event_type == 0) event_type = PROC_SOFT_EVENT_IPC_FIFO;
    return proc_soft_event(target_pid, event_type,
                           (flags & PROC_SW_INT_F_BOOST) != 0) ?
           PROC_IPC_OK : PROC_IPC_ERR_NOENT;
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
    if (!ptr_valid(c, sizeof(tensor_t))) return -1;
    if (!ptr_valid(a, sizeof(tensor_t))) return -1;
    if (!ptr_valid(b, sizeof(tensor_t))) return -1;
    return tensor_matmul((tensor_t *)c, (const tensor_t *)a, (const tensor_t *)b) ? 0 : -1;
}

static i32 sys_tensor_relu(void *b, const void *a)
{
    if (!ptr_valid(b, sizeof(tensor_t))) return -1;
    if (!ptr_valid(a, sizeof(tensor_t))) return -1;
    return tensor_relu((tensor_t *)b, (const tensor_t *)a) ? 0 : -1;
}

static i32 sys_tensor_softmax(void *b, const void *a)
{
    if (!ptr_valid(b, sizeof(tensor_t))) return -1;
    if (!ptr_valid(a, sizeof(tensor_t))) return -1;
    return tensor_softmax((tensor_t *)b, (const tensor_t *)a) ? 0 : -1;
}

static i32 sys_tensor_add(void *c, const void *a, const void *b)
{
    if (!ptr_valid(c, sizeof(tensor_t))) return -1;
    if (!ptr_valid(a, sizeof(tensor_t))) return -1;
    if (!ptr_valid(b, sizeof(tensor_t))) return -1;
    return tensor_add((tensor_t *)c, (const tensor_t *)a, (const tensor_t *)b) ? 0 : -1;
}

static i32 sys_tensor_dot(void *result, const void *a, const void *b)
{
    if (!ptr_valid(result, sizeof(float))) return -1;
    return tensor_dot((float *)result, (const tensor_t *)a, (const tensor_t *)b) ? 0 : -1;
}

static i32 sys_tensor_mul(void *c, const void *a, const void *b)
{
    if (!ptr_valid(c, sizeof(tensor_t))) return -1;
    if (!ptr_valid(a, sizeof(tensor_t))) return -1;
    if (!ptr_valid(b, sizeof(tensor_t))) return -1;
    return tensor_mul((tensor_t *)c, (const tensor_t *)a, (const tensor_t *)b) ? 0 : -1;
}

static i32 sys_tensor_scale(void *b, const void *a, float scalar)
{
    if (!ptr_valid(b, sizeof(tensor_t))) return -1;
    if (!ptr_valid(a, sizeof(tensor_t))) return -1;
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
