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

#define CORE_DISK  1

static struct process  procs[MAX_PROCS_PER_CORE];
static struct proc_context scheduler_ctx;
static u32 current_proc;   /* index into procs[] */
static u32 next_pid;
static bool initialized;
static u64 heap_top[MAX_PROCS_PER_CORE];

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
static i32   sys_read(i32 fd, void *buf, u32 len);
static i32   sys_write(i32 fd, const void *buf, u32 len);
static i32   sys_close(i32 fd);
static i32   sys_stat(const char *path, void *out);
static i32   sys_mkdir(const char *path);
static i32   sys_unlink(const char *path);
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
static i32   sys_tensor_alloc(void *t, u32 rows, u32 cols, u32 elem_size);
static void  sys_tensor_free(void *t);
static void  sys_tensor_upload(void *t, const void *data);
static void  sys_tensor_download(const void *t, void *data);
static i32   sys_tensor_matmul(void *c, const void *a, const void *b);
static i32   sys_tensor_relu(void *b, const void *a);
static i32   sys_tensor_softmax(void *b, const void *a);
static i32   sys_tensor_add(void *c, const void *a, const void *b);
static i32   sys_tensor_dot(void *result, const void *a, const void *b);

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
    .read            = sys_read,
    .write           = sys_write,
    .close           = sys_close,
    .stat            = sys_stat,
    .mkdir           = sys_mkdir,
    .unlink          = sys_unlink,
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

    heap_top[(u32)slot] = ((u64)(usize)base + loaded + 15) & ~15UL;

    simd_zero(&p->ctx, sizeof(p->ctx));
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

static i32 sys_close(i32 fd) { (void)fd; return 0; }

static i32 sys_stat(const char *path, void *out)
{
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
    if (!ptr_valid(path, 1)) return -1;
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
    if (!ptr_valid(path, 1)) return -1;
    struct fifo_msg msg = {0};
    msg.type   = MSG_FS_DELETE;
    msg.buffer = (u64)(usize)path;
    msg.length = pios_strlen(path) + 1;
    struct fifo_msg reply;
    fs_request(&msg, &reply);
    return reply.status == 0 ? 0 : -1;
}

/* ---- Framebuffer ---- */

static void sys_fb_putc(char c)                    { fb_putc(c); }
static void sys_fb_print(const char *s)            { if (ptr_valid(s, 1)) fb_puts(s); }
static void sys_fb_color(u32 fg, u32 bg)           { fb_set_color(fg, bg); }
static void sys_fb_clear(u32 color)                { fb_clear(color); }
static void sys_fb_pixel(u32 x, u32 y, u32 color)  { fb_pixel(x, y, color); }

/* ---- Networking ---- */

static i32 sys_socket(u32 type) { return sock_socket(type); }

static i32 sys_bind(i32 fd, u32 ip, u16 port)
{
    struct sockaddr_in addr = { .ip = ip, .port = port };
    return sock_bind(fd, &addr);
}

static i32 sys_connect(i32 fd, u32 ip, u16 port)
{
    struct sockaddr_in addr = { .ip = ip, .port = port };
    return sock_connect(fd, &addr);
}

static i32 sys_listen(i32 fd, u32 backlog) { return sock_listen(fd, backlog); }

static i32 sys_accept(i32 fd, u32 *client_ip, u16 *client_port)
{
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
    if (!ptr_valid(data, len)) return -1;
    return sock_send(fd, data, len);
}

static i32 sys_recv(i32 fd, void *buf, u32 len)
{
    if (!ptr_valid(buf, len)) return -1;
    return sock_recv(fd, buf, len);
}

static i32 sys_sendto(i32 fd, const void *data, u32 len, u32 ip, u16 port)
{
    if (!ptr_valid(data, len)) return -1;
    struct sockaddr_in dest = { .ip = ip, .port = port };
    return sock_sendto(fd, data, len, &dest);
}

static i32 sys_recvfrom(i32 fd, void *buf, u32 len, u32 *src_ip, u16 *src_port)
{
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

static i32 sys_sock_close(i32 fd) { return sock_close(fd); }

/* ---- DNS ---- */

static i32 sys_resolve(const char *hostname, u32 *ip_out)
{
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
