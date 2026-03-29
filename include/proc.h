/*
 * proc.h - Cooperative process manager for user cores (2-3)
 *
 * Each user core runs an independent scheduler with up to 6 processes.
 * Processes occupy fixed 2MB slots within the core's 16MB private RAM.
 * Scheduling is cooperative: processes yield voluntarily via the syscall table.
 */

#pragma once
#include "types.h"

#define MAX_PROCS_PER_CORE  6
#define PROC_SLOT_SIZE      (2 * 1024 * 1024)   /* 2MB per process */
#define PROC_SLOT_OFFSET    0x100000             /* slots start 1MB into core RAM */

/* Process states */
#define PROC_EMPTY    0
#define PROC_READY    1
#define PROC_RUNNING  2
#define PROC_BLOCKED  3
#define PROC_DEAD     4

/* Saved context for cooperative context switch (callee-saved only) */
struct proc_context {
    u64 x19_x30[12];   /* x19-x30 (12 callee-saved registers) */
    u64 sp;
} ALIGNED(16);

struct process {
    u32 pid;
    u32 state;
    u32 principal_id;
    u8 *base;           /* 2MB slot start */
    u32 mem_size;
    struct proc_context ctx;
    u64 ticks;          /* tick count at last schedule */
    u32 exit_code;
};

/* Syscall table passed to processes in x0 at entry.
 * This is the COMPLETE userland API surface.
 * All pointers passed by userland are bounds-checked. */
struct syscall_table {
    /* ---- Process control ---- */
    i32 (*yield)(void);
    i32 (*exit)(u32 code);
    u32 (*getpid)(void);

    /* ---- Console I/O ---- */
    void (*print)(const char *msg);
    void (*putc)(char c);
    i32  (*getc)(void);             /* blocking keyboard read */
    i32  (*try_getc)(void);         /* non-blocking, returns -1 if none */

    /* ---- Timer ---- */
    u64 (*ticks)(void);
    void (*sleep_ms)(u64 ms);
    void (*sleep_us)(u64 us);

    /* ---- Filesystem (WALFS via FIFO to Core 1) ---- */
    i32 (*open)(const char *path, u32 flags);
    i32 (*creat)(const char *path, u32 flags, u32 mode); /* create new file/dir */
    i32 (*read)(i32 fd, void *buf, u32 len);
    i32 (*write)(i32 fd, const void *buf, u32 len);
    i32 (*pread)(i32 fd, void *buf, u32 len, u64 offset);  /* positioned read */
    i32 (*pwrite)(i32 fd, const void *buf, u32 len, u64 offset); /* positioned write */
    i32 (*close)(i32 fd);
    i32 (*stat)(const char *path, void *out);  /* fills walfs_inode */
    i32 (*mkdir)(const char *path);
    i32 (*unlink)(const char *path);
    i32 (*readdir)(const char *path, void *entries, u32 max_entries); /* list directory */

    /* ---- Framebuffer ---- */
    void (*fb_putc)(char c);
    void (*fb_print)(const char *s);
    void (*fb_color)(u32 fg, u32 bg);
    void (*fb_clear)(u32 color);
    void (*fb_pixel)(u32 x, u32 y, u32 color);

    /* ---- Networking: sockets ---- */
    i32 (*socket)(u32 type);
    i32 (*bind)(i32 fd, u32 ip, u16 port);
    i32 (*connect)(i32 fd, u32 ip, u16 port);
    i32 (*listen)(i32 fd, u32 backlog);
    i32 (*accept)(i32 fd, u32 *client_ip, u16 *client_port);
    i32 (*send)(i32 fd, const void *data, u32 len);
    i32 (*recv)(i32 fd, void *buf, u32 len);
    i32 (*sendto)(i32 fd, const void *data, u32 len, u32 ip, u16 port);
    i32 (*recvfrom)(i32 fd, void *buf, u32 len, u32 *src_ip, u16 *src_port);
    i32 (*sock_close)(i32 fd);

    /* ---- DNS ---- */
    i32 (*resolve)(const char *hostname, u32 *ip_out);

    /* ---- Identity ---- */
    u32 (*whoami)(void);
    i32 (*auth)(const char *user, const char *pass);

    /* ---- Memory ---- */
    void *(*sbrk)(i32 increment);
    void *(*memset)(void *dst, i32 c, u32 n);
    void *(*memcpy)(void *dst, const void *src, u32 n);
    u32   (*strlen)(const char *s);

    /* ---- Process management ---- */
    i32 (*spawn)(const char *path);     /* load + start child process */
    i32 (*wait)(i32 pid);               /* wait for child to exit, return exit code */
    u32 (*nprocs)(void);                /* count active processes on this core */

    /* ---- Inter-process sync ---- */
    i32 (*sem_create)(u32 initial);     /* create semaphore, returns id */
    i32 (*sem_wait)(i32 id);            /* decrement (blocks if 0) */
    i32 (*sem_post)(i32 id);            /* increment */

    /* ---- In-memory IPC ---- */
    i32 (*queue_create)(const char *name, u32 depth, u32 flags, u32 frame_max);
    i32 (*queue_push)(i32 qid, const void *data, u32 len);
    i32 (*queue_pop)(i32 qid, void *out, u32 out_max);      /* returns frame len */
    i32 (*queue_len)(i32 qid);
    i32 (*stack_create)(const char *name, u32 depth, u32 flags, u32 frame_max);
    i32 (*stack_push)(i32 sid, const void *data, u32 len);
    i32 (*stack_pop)(i32 sid, void *out, u32 out_max);      /* returns frame len */
    i32 (*stack_len)(i32 sid);
    i32 (*topic_create)(const char *name, u32 replay_window, u32 flags, u32 event_max);
    i32 (*topic_publish)(i32 tid, const void *data, u32 len);
    i32 (*topic_subscribe)(i32 tid);                        /* returns subscriber handle */
    i32 (*topic_read)(i32 sub_id, void *out, u32 out_max); /* returns event len */

    /* ---- Tensor / GPU compute ---- */
    i32 (*tensor_alloc)(void *t, u32 rows, u32 cols, u32 elem_size);
    void (*tensor_free)(void *t);
    void (*tensor_upload)(void *t, const void *data);
    void (*tensor_download)(const void *t, void *data);
    i32 (*tensor_matmul)(void *c, const void *a, const void *b);
    i32 (*tensor_relu)(void *b, const void *a);
    i32 (*tensor_softmax)(void *b, const void *a);
    i32 (*tensor_add)(void *c, const void *a, const void *b);
    i32 (*tensor_dot)(void *result, const void *a, const void *b);
};

/* Assembly context switch: save callee-saved regs to old, restore from new */
extern void ctx_switch(struct proc_context *old, struct proc_context *new_ctx);

void proc_init(void);              /* init scheduler on this core */
i32  proc_exec(const char *path);  /* load + start process from WALFS */
void proc_yield(void);             /* cooperative context switch */
NORETURN void proc_exit(u32 code); /* terminate current process */
void proc_schedule(void);          /* run scheduler loop (called from coreN_main) */
u32  proc_count(void);             /* number of active processes on this core */
bool proc_handle_fault(u64 esr, u64 elr, u64 far); /* kill faulting user proc */
