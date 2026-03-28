/* proc.c - Cooperative process manager for user cores (2-3) */

#include "proc.h"
#include "core.h"
#include "core_env.h"
#include "walfs.h"
#include "principal.h"
#include "uart.h"
#include "timer.h"
#include "socket.h"

static struct process  procs[MAX_PROCS_PER_CORE];
static struct proc_context scheduler_ctx;
static u32 current_proc;   /* index into procs[] */
static u32 next_pid;
static bool initialized;

/* Validate a user pointer is within the current process's memory slot */
static bool ptr_valid(const void *ptr, u32 len) {
    struct process *p = &procs[current_proc];
    u64 addr = (u64)(usize)ptr;
    u64 end = addr + len;
    u64 slot_start = (u64)(usize)p->base;
    u64 slot_end = slot_start + p->mem_size;
    return addr >= slot_start && end <= slot_end && end >= addr;
}

static i32 sys_yield(void);
static i32 sys_exit(u32 code);
static i32 sys_open(const char *path, u32 flags);
static i32 sys_read(i32 fd, void *buf, u32 len);
static i32 sys_write(i32 fd, const void *buf, u32 len);
static i32 sys_close(i32 fd);
static void sys_print(const char *msg);
static u64 sys_ticks(void);
static i32 sys_socket(u32 type);
static i32 sys_connect(i32 fd, u32 ip, u16 port);
static i32 sys_send(i32 fd, const void *data, u32 len);
static i32 sys_recv(i32 fd, void *buf, u32 len);

static struct syscall_table syscall_tab = {
    .yield   = sys_yield,
    .exit    = sys_exit,
    .open    = sys_open,
    .read    = sys_read,
    .write   = sys_write,
    .close   = sys_close,
    .print   = sys_print,
    .ticks   = sys_ticks,
    .socket  = sys_socket,
    .connect = sys_connect,
    .send    = sys_send,
    .recv    = sys_recv,
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

void proc_init(void)
{
    for (u32 i = 0; i < MAX_PROCS_PER_CORE; i++) {
        procs[i].state = PROC_EMPTY;
        procs[i].pid = 0;
    }
    current_proc = 0;
    next_pid = (core_id() << 16) | 1;  /* encode core in upper bits */
    initialized = true;
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
    memset(base, 0, PROC_SLOT_SIZE);
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

    memset(&p->ctx, 0, sizeof(p->ctx));
    p->ctx.x19_x30[0] = (u64)(usize)&syscall_tab;  /* x19 */
    p->ctx.x19_x30[1] = (u64)(usize)base;           /* x20 */
    p->ctx.x19_x30[11] = (u64)(usize)proc_trampoline; /* x30 = LR */
    p->ctx.sp = (u64)(usize)(base + PROC_SLOT_SIZE - 16);

    uart_puts("[proc] loaded pid=");
    uart_hex(p->pid);
    uart_puts(" path=");
    uart_puts(path);
    uart_putc('\n');

    return (i32)p->pid;
}

void proc_yield(void)
{
    struct process *p = &procs[current_proc];
    if (p->state == PROC_RUNNING)
        p->state = PROC_READY;
    ctx_switch(&p->ctx, &scheduler_ctx);
}

NORETURN void proc_exit(u32 code)
{
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

            ctx_switch(&scheduler_ctx, &procs[i].ctx);
            /* Returns here when process yields or exits */
        }

        if (!found)
            wfe();
    }
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

/* Stubs — filesystem via FIFO is complex, implement later */
static i32 sys_open(const char *p, u32 f)  { (void)p; (void)f; return -1; }
static i32 sys_read(i32 d, void *b, u32 l) { (void)d; (void)b; (void)l; return -1; }
static i32 sys_write(i32 d, const void *b, u32 l) { (void)d; (void)b; (void)l; return -1; }
static i32 sys_close(i32 d) { (void)d; return -1; }

static void sys_print(const char *msg)
{
    if (!ptr_valid(msg, 1)) return;
    uart_puts(msg);
}

static u64 sys_ticks(void)
{
    return timer_ticks();
}

static i32 sys_socket(u32 type)
{
    return sock_socket(type);
}

static i32 sys_connect(i32 fd, u32 ip, u16 port)
{
    struct sockaddr_in addr;
    addr.ip = ip;
    addr.port = port;
    return sock_connect(fd, &addr);
}

static i32 sys_send(i32 fd, const void *data, u32 len)
{
    if (!ptr_valid(data, len)) return -1;
    return sock_send(fd, data, len);
}

static i32 sys_recv(i32 fd, void *buf, u32 len)
{
    if (!ptr_valid(buf, len)) return -1;
    return sock_recv(fd, buf, len);
}
