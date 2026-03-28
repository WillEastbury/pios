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

/* Syscall table passed to processes in x0 */
struct syscall_table {
    i32 (*yield)(void);
    i32 (*exit)(u32 code);
    i32 (*open)(const char *path, u32 flags);
    i32 (*read)(i32 fd, void *buf, u32 len);
    i32 (*write)(i32 fd, const void *buf, u32 len);
    i32 (*close)(i32 fd);
    void (*print)(const char *msg);
    u64 (*ticks)(void);
    i32 (*socket)(u32 type);
    i32 (*connect)(i32 fd, u32 ip, u16 port);
    i32 (*send)(i32 fd, const void *data, u32 len);
    i32 (*recv)(i32 fd, void *buf, u32 len);
};

/* Assembly context switch: save callee-saved regs to old, restore from new */
extern void ctx_switch(struct proc_context *old, struct proc_context *new_ctx);

void proc_init(void);              /* init scheduler on this core */
i32  proc_exec(const char *path);  /* load + start process from WALFS */
void proc_yield(void);             /* cooperative context switch */
NORETURN void proc_exit(u32 code); /* terminate current process */
void proc_schedule(void);          /* run scheduler loop (called from coreN_main) */
u32  proc_count(void);             /* number of active processes on this core */
