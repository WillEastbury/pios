/* proc.c - Cooperative process manager for user cores (2-3) */

#include "proc.h"
#include "core.h"
#include "core_env.h"
#include "walfs.h"
#include "principal.h"
#include "uart.h"
#include "timer.h"
#include "socket.h"
#include "dma.h"
#include "simd.h"
#include "fb.h"
#include "usb_kbd.h"
#include "dns.h"
#include "tensor.h"
#include "fifo.h"
#include "mmu.h"
#include "ipc_queue.h"
#include "ipc_stream.h"
#include "pipe.h"
#include "exception.h"

#define CORE_DISK  1

static struct process  procs[MAX_PROCS_PER_CORE];
static struct proc_context scheduler_ctx;
static u32 current_proc;   /* index into procs[] */
static u32 next_pid;
static bool initialized;
static u64 heap_top[MAX_PROCS_PER_CORE];
static volatile bool preempt_enabled[2];
static volatile bool preempt_armed[2];
static volatile bool preempt_pending[2];
static u64 preempt_quantum_ticks[2];
static u64 preempt_count_core[2];

static inline bool on_user_core(void)
{
    u32 c = core_id();
    return c == CORE_USER0 || c == CORE_USER1;
}

static inline u32 user_core_slot(void)
{
    return core_id() - CORE_USER0;
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

/* ---- Semaphore state ---- */
#define MAX_SEMS 8
static struct { bool used; volatile i32 count; } sems[MAX_SEMS];

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
static i32   sys_tensor_alloc(void *t, u32 rows, u32 cols, u32 elem_size);
static void  sys_tensor_free(void *t);
static void  sys_tensor_upload(void *t, const void *data);
static void  sys_tensor_download(const void *t, void *data);
static i32   sys_tensor_matmul(void *c, const void *a, const void *b);
static i32   sys_tensor_relu(void *b, const void *a);
static i32   sys_tensor_softmax(void *b, const void *a);
static i32   sys_tensor_add(void *c, const void *a, const void *b);
static i32   sys_tensor_dot(void *result, const void *a, const void *b);
static void  proc_tick_hook(u32 core, u64 tick);
static void  proc_preempt_trampoline(void);

static struct syscall_table syscall_tab = {
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

/* Trampoline: x19=&syscall_tab, x20=entry. First schedule lands here via LR. */
static NORETURN void proc_trampoline(void)
{
    struct syscall_table *tab;
    u64 entry;
    __asm__ volatile("mov %0, x19" : "=r"(tab));
    __asm__ volatile("mov %0, x20" : "=r"(entry));

    ((void (*)(struct syscall_table *))entry)(tab);
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
    current_proc = 0;
    next_pid = (core_id() << 16) | 1;  /* encode core in upper bits */
    initialized = true;
    if (on_user_core()) {
        preempt_enabled[uc] = false;
        preempt_armed[uc] = false;
        preempt_pending[uc] = false;
        preempt_quantum_ticks[uc] = 1;
        preempt_count_core[uc] = 0;
    }
    mmu_switch_to_kernel();
}

i32 proc_exec(const char *path)
{
    if (!initialized)
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

    struct process *p = &procs[slot];
    p->pid = next_pid++;
    p->state = PROC_READY;
    p->principal_id = principal_current();
    p->base = base;
    p->mem_size = PROC_SLOT_SIZE;
    p->ticks = timer_ticks();
    p->exit_code = 0;
    p->preemptions = 0;

    heap_top[(u32)slot] = ((u64)(usize)base + loaded + 15) & ~15UL;

    simd_zero(&p->ctx, sizeof(p->ctx));
    p->ctx.x19_x30[0] = (u64)(usize)&syscall_tab;  /* x19 */
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

void proc_yield(void)
{
    bool user = on_user_core();
    u32 uc = user ? user_core_slot() : 0;
    struct process *p = &procs[current_proc];
    if (user) {
        preempt_armed[uc] = false;
        preempt_pending[uc] = false;
    }
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
        bool found = false;

        for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
            if (procs[i].state == PROC_DEAD) {
                procs[i].state = PROC_EMPTY;
                procs[i].pid = 0;
                continue;
            }
            if (procs[i].state != PROC_READY)
                continue;

            found = true;
            current_proc = i;
            procs[i].state = PROC_RUNNING;
            procs[i].ticks = timer_ticks();
            principal_set_current(procs[i].principal_id);
            if (!mmu_switch_to_user(core_id(), i)) {
                procs[i].state = PROC_DEAD;
                procs[i].exit_code = 0xFFFF0002U;
                mmu_switch_to_kernel();
                principal_set_current(PRINCIPAL_ROOT);
                continue;
            }

            if (user && preempt_enabled[uc]) {
                preempt_pending[uc] = false;
                preempt_armed[uc] = true;
            }
            ctx_switch(&scheduler_ctx, &procs[i].ctx);
            /* Returns here when process yields or exits */
            if (user) {
                preempt_armed[uc] = false;
                preempt_pending[uc] = false;
            }
            mmu_switch_to_kernel();
            principal_set_current(PRINCIPAL_ROOT);
        }

        if (!found)
            wfe();
    }
}

bool proc_handle_fault(u64 esr, u64 elr, u64 far)
{
    if (!initialized)
        return false;
    if ((core_id() != CORE_USER0 && core_id() != CORE_USER1))
        return false;
    u32 uc = user_core_slot();
    struct process *p = &procs[current_proc];
    if (p->state != PROC_RUNNING)
        return false;

    preempt_armed[uc] = false;
    preempt_pending[uc] = false;
    p->state = PROC_DEAD;
    p->exit_code = 0xFFFF0001U;
    mmu_switch_to_kernel();
    principal_set_current(PRINCIPAL_ROOT);

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
    if (core != CORE_USER0 && core != CORE_USER1)
        return;

    u32 uc = core - CORE_USER0;
    if (!preempt_enabled[uc] || !preempt_armed[uc] || preempt_pending[uc])
        return;

    struct process *p = &procs[current_proc];
    if (p->state != PROC_RUNNING)
        return;

    u64 elapsed = tick - p->ticks;
    if (elapsed >= preempt_quantum_ticks[uc])
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

/* ---- Filesystem (WALFS via FIFO to Core 1) ---- */

static i32 sys_open(const char *path, u32 flags)
{
    (void)flags;
    if (!has_disk_cap()) return -1;
    if (!ptr_valid(path, 1)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_FIND;
    msg.buffer = (u64)(usize)path;
    msg.length = pios_strlen(path) + 1;
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
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_WRITE;
    msg.param  = (u32)fd;
    msg.buffer = (u64)(usize)buf;
    msg.length = len;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return (i32)reply.length;
}

static i32 sys_close(i32 fd) { if (!has_disk_cap()) return -1; (void)fd; return 0; }

static i32 sys_stat(const char *path, void *out)
{
    if (!has_disk_cap()) return -1;
    if (!ptr_valid(path, 1)) return -1;
    if (!ptr_valid(out, sizeof(struct walfs_inode))) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_STAT;
    msg.buffer = (u64)(usize)path;
    msg.length = pios_strlen(path) + 1;
    msg.tag    = (u64)(usize)out;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    if (reply.status != 0) return -1;
    return 0;
}

static i32 sys_mkdir(const char *path)
{
    if (!ptr_valid(path, 1) || !has_disk_cap()) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_MKDIR;
    msg.buffer = (u64)(usize)path;
    msg.length = pios_strlen(path) + 1;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    return reply.status == 0 ? 0 : -1;
}

static i32 sys_unlink(const char *path)
{
    if (!ptr_valid(path, 1) || !has_disk_cap()) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_DELETE;
    msg.buffer = (u64)(usize)path;
    msg.length = pios_strlen(path) + 1;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    return reply.status == 0 ? 0 : -1;
}

static i32 sys_creat(const char *path, u32 flags, u32 mode)
{
    if (!ptr_valid(path, 1) || !has_disk_cap()) return -1;

    /* Split path into parent directory and filename */
    u32 len = pios_strlen(path);
    if (len == 0) return -1;

    /* Find last '/' to separate parent path from filename */
    i32 sep = -1;
    for (u32 i = 0; i < len; i++) {
        if (path[i] == '/')
            sep = (i32)i;
    }

    /* Resolve parent directory inode */
    u64 parent_inode;
    if (sep <= 0) {
        /* File in root directory — find "/" */
        struct fifo_msg fmsg = {0};
        fmsg.type   = MSG_FS_FIND;
        fmsg.buffer = (u64)(usize)"/";
        fmsg.length = 2;
        struct fifo_msg freply;
        fs_request(&fmsg, &freply);
        if (freply.status != 0) return -1;
        parent_inode = freply.param;
    } else {
        /* Build parent path on the stack (reuse the buffer up to sep) */
        struct fifo_msg fmsg = {0};
        fmsg.type   = MSG_FS_FIND;
        fmsg.buffer = (u64)(usize)path;
        fmsg.length = (u32)sep + 1; /* length up to but not including sep slash */
        fmsg.tag    = (u64)(u32)sep; /* hint: path length to consider */
        struct fifo_msg freply;
        fs_request(&fmsg, &freply);
        if (freply.status != 0) return -1;
        parent_inode = freply.param;
    }

    /* Create file under parent */
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_CREATE;
    msg.param  = (u32)parent_inode;
    msg.buffer = (u64)(usize)(path + sep + 1); /* filename portion */
    msg.length = len - (u32)(sep + 1) + 1;     /* include null terminator */
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
    if (!ptr_valid(path, 1)) return -1;
    if (!ptr_valid(entries, max_entries * sizeof(struct readdir_entry))) return -1;

    /* Resolve path to inode */
    struct fifo_msg fmsg = {0};
    fmsg.type   = MSG_FS_FIND;
    fmsg.buffer = (u64)(usize)path;
    fmsg.length = pios_strlen(path) + 1;
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

/* ---- Framebuffer ---- */

static void sys_fb_putc(char c)                    { fb_putc(c); }
static void sys_fb_print(const char *s)            { if (ptr_valid(s, 1)) fb_puts(s); }
static void sys_fb_color(u32 fg, u32 bg)           { fb_set_color(fg, bg); }
static void sys_fb_clear(u32 color)                { fb_clear(color); }
static void sys_fb_pixel(u32 x, u32 y, u32 color)  { fb_pixel(x, y, color); }

/* ---- Networking ---- */

static i32 sys_socket(u32 type) { if (!has_net_cap()) return -1; return sock_socket(type); }

static i32 sys_bind(i32 fd, u32 ip, u16 port)
{
    if (!has_net_cap()) return -1;
    struct sockaddr_in addr = { .ip = ip, .port = port };
    return sock_bind(fd, &addr);
}

static i32 sys_connect(i32 fd, u32 ip, u16 port)
{
    if (!has_net_cap()) return -1;
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

static i32 sys_sock_close(i32 fd) { if (!has_net_cap()) return -1; return sock_close(fd); }

/* ---- DNS ---- */

static i32 sys_resolve(const char *hostname, u32 *ip_out)
{
    if (!has_net_cap()) return -1;
    if (!ptr_valid(hostname, 1)) return -1;
    if (!ptr_valid(ip_out, sizeof(u32))) return -1;
    return dns_resolve(hostname, ip_out) ? 0 : -1;
}

/* ---- Identity ---- */

static u32 sys_whoami(void) { return principal_current(); }

static i32 sys_auth(const char *user, const char *pass)
{
    if (!ptr_valid(user, 1)) return -1;
    if (!ptr_valid(pass, 1)) return -1;
    return principal_auth(user, pass, NULL) ? 0 : -1;
}

/* ---- Memory ---- */

static void *sys_sbrk(i32 increment)
{
    struct process *p = &procs[current_proc];
    u64 old = heap_top[current_proc];
    u64 new_top = old + increment;
    u64 limit = (u64)(usize)p->base + p->mem_size - 65536;
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
    if (!ptr_valid(s, 1)) return 0;
    return pios_strlen(s);
}

/* ---- Process management ---- */

static i32 sys_spawn(const char *path)
{
    if (!ptr_valid(path, 1) || !has_cap(PRINCIPAL_EXEC)) return -1;
    return proc_exec(path);
}

static i32 sys_wait(i32 pid)
{
    for (;;) {
        for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
            if (procs[i].pid == (u32)pid && procs[i].state == PROC_DEAD)
                return (i32)procs[i].exit_code;
        }
        proc_yield();
    }
}

static u32 sys_nprocs(void) { return proc_count(); }

/* ---- Semaphores ---- */

static i32 sys_sem_create(u32 initial)
{
    for (u32 i = 0; i < MAX_SEMS; i++) {
        if (!sems[i].used) {
            sems[i].used = true;
            sems[i].count = (i32)initial;
            return (i32)i;
        }
    }
    return -1;
}

static i32 sys_sem_wait(i32 id)
{
    if (id < 0 || id >= MAX_SEMS || !sems[id].used) return -1;
    while (sems[id].count <= 0)
        proc_yield();
    sems[id].count--;
    return 0;
}

static i32 sys_sem_post(i32 id)
{
    if (id < 0 || id >= MAX_SEMS || !sems[id].used) return -1;
    sems[id].count++;
    return 0;
}

/* ---- In-memory IPC ---- */

static bool ptr_valid_cstr(const char *s, u32 max_len)
{
    if (!s || max_len == 0) return false;
    for (u32 i = 0; i < max_len; i++) {
        if (!ptr_valid(s + i, 1)) return false;
        if (s[i] == 0) return i != 0;
    }
    return false;
}

static i32 sys_queue_create(const char *name, u32 depth, u32 flags, u32 frame_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid_cstr(name, IPC_NAME_MAX + 1)) return -1;
    return ipc_queue_create(name, depth, flags, frame_max);
}

static i32 sys_queue_push(i32 qid, const void *data, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(data, len)) return -1;
    return ipc_queue_push(qid, data, len);
}

static i32 sys_queue_pop(i32 qid, void *out, u32 out_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(out, out_max)) return -1;
    u32 len = 0;
    i32 r = ipc_queue_pop(qid, out, out_max, &len);
    if (r != IPC_OK) return r;
    return (i32)len;
}

static i32 sys_queue_len(i32 qid)
{
    if (!has_ipc_cap()) return -1;
    return ipc_queue_len(qid);
}

static i32 sys_stack_create(const char *name, u32 depth, u32 flags, u32 frame_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid_cstr(name, IPC_NAME_MAX + 1)) return -1;
    return ipc_stack_create(name, depth, flags, frame_max);
}

static i32 sys_stack_push(i32 sid, const void *data, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(data, len)) return -1;
    return ipc_stack_push(sid, data, len);
}

static i32 sys_stack_pop(i32 sid, void *out, u32 out_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(out, out_max)) return -1;
    u32 len = 0;
    i32 r = ipc_stack_pop(sid, out, out_max, &len);
    if (r != IPC_OK) return r;
    return (i32)len;
}

static i32 sys_stack_len(i32 sid)
{
    if (!has_ipc_cap()) return -1;
    return ipc_stack_len(sid);
}

static i32 sys_topic_create(const char *name, u32 replay_window, u32 flags, u32 event_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid_cstr(name, IPC_NAME_MAX + 1)) return -1;
    return ipc_topic_create(name, replay_window, flags, event_max);
}

static i32 sys_topic_publish(i32 tid, const void *data, u32 len)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(data, len)) return -1;
    return ipc_topic_publish(tid, data, len);
}

static i32 sys_topic_subscribe(i32 tid)
{
    if (!has_ipc_cap()) return -1;
    return ipc_topic_subscribe(tid);
}

static i32 sys_topic_read(i32 sub_id, void *out, u32 out_max)
{
    if (!has_ipc_cap()) return -1;
    if (!ptr_valid(out, out_max)) return -1;
    u32 len = 0;
    i32 r = ipc_topic_read(sub_id, out, out_max, &len);
    if (r != IPC_OK) return r;
    return (i32)len;
}

/* ---- Unified pipes (/ipc functional, net/fs/hw explicit stubs) ---- */

static bool path_is_ipc(const char *path)
{
    return path &&
           path[0] == '/' &&
           path[1] == 'i' &&
           path[2] == 'p' &&
           path[3] == 'c' &&
           path[4] == '/';
}

static i32 sys_pipe_create(const char *path, u32 type, u32 depth, u32 flags, u32 frame_max)
{
    if (!ptr_valid_cstr(path, PIPE_PATH_MAX + 1)) return -1;
    if (path_is_ipc(path) && !has_ipc_cap()) return -1;
    return pipe_create(path, type, depth, flags, frame_max);
}

static i32 sys_pipe_open(const char *path, u32 type)
{
    if (!ptr_valid_cstr(path, PIPE_PATH_MAX + 1)) return -1;
    if (path_is_ipc(path) && !has_ipc_cap()) return -1;
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
