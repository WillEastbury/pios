/*
 * proc.h - Cooperative process manager for user cores (2-3)
 *
 * Each user core runs an independent scheduler with up to 6 processes.
 * Processes occupy fixed 2MB slots within the core's 16MB private RAM.
 * Scheduling is preemptive on user cores via timer quanta and cooperative yield.
 */

#pragma once
#include "types.h"
#include "pipe.h"
struct irq_frame;

struct paged_io_stat {
    u64 inode_id;
    u64 file_size;
    u32 page_size;
    u32 flags;
} PACKED;

struct proc_ui_entry {
    u32 pid;
    u32 principal_id;
    u32 state;
    u32 affinity_core;
    u32 priority_class;
    u32 mem_kib;
    u32 cpu_percent;
    u32 preemptions;
    u64 runtime_ticks;
} PACKED;

struct proc_capsule_ui_entry {
    u32 pid;
    u32 principal_id;
    u32 state;
    u32 affinity_core;
    u32 capsule_id;
    u32 capsule_hash;
    char group[32];
    char vfs_root[96];
} PACKED;

#define APPF_EVENT_DATA_MAX   224U
#define APPF_LOG_MSG_MAX      224U
#define APPF_SERVICE_NAME_MAX 31U

struct appf_event_record {
    u32 seq;
    u32 type;
    u32 len;
    u8  data[APPF_EVENT_DATA_MAX];
} PACKED;

struct appf_log_record {
    u32 seq;
    u32 level;
    u32 len;
    u8  msg[APPF_LOG_MSG_MAX];
} PACKED;

struct appf_service_record {
    char name[APPF_SERVICE_NAME_MAX + 1];
    u32 kind;
    u32 endpoint;
    u32 owner_principal;
    u32 flags;
} PACKED;

#define MAX_PROCS_PER_CORE  6
#define PROC_SLOT_SIZE      (2 * 1024 * 1024)   /* 2MB per process */
#define PROC_SLOT_OFFSET    0x100000             /* slots start 1MB into core RAM */

/* Process states */
#define PROC_EMPTY    0
#define PROC_READY    1
#define PROC_RUNNING  2
#define PROC_BLOCKED  3
#define PROC_DEAD     4

/* Priority classes (low preempt interval = more frequent scheduling) */
#define PROC_PRIO_LAZY      0
#define PROC_PRIO_LOW       1
#define PROC_PRIO_NORMAL    2
#define PROC_PRIO_HIGH      3
#define PROC_PRIO_REALTIME  4
#define PROC_CAPSULE_ID_NONE 0xFFFFFFFFU

/* Saved context for cooperative context switch (callee-saved only) */
struct proc_context {
    u64 x19_x30[12];   /* x19-x30 (12 callee-saved registers) */
    u64 sp;
} ALIGNED(16);

struct process {
    u32 pid;
    u32 parent_pid;
    u32 state;
    u32 principal_id;
    u32 affinity_core;
    u32 priority_class;
    u64 quantum_ticks;
    u8 *base;           /* 2MB slot start */
    u32 mem_size;
    struct proc_context ctx;
    u64 ticks;          /* tick count at last schedule */
    u64 runtime_ticks;  /* accumulated runtime ticks */
    u32 exit_code;
    u32 preemptions;
    bool capsule_enabled;
    u32 capsule_manifest_hash;
    bool capsule_allow_spawn;
    bool capsule_allow_wait;
    bool capsule_allow_nprocs;
    char capsule_group[32];
    char capsule_vfs_root[96];
    u32 capsule_fs_prefix_count;
    char capsule_fs_prefix[8][64];
    u32 capsule_ipc_prefix_count;
    char capsule_ipc_prefix[8][32];
    u32 capsule_pipe_prefix_count;
    char capsule_pipe_prefix[8][64];
    u32 capsule_card_range_count;
    struct {
        u16 lo;
        u16 hi;
    } capsule_card_ranges[8];
    u32 capsule_port_range_count;
    struct {
        u16 lo;
        u16 hi;
    } capsule_port_ranges[8];
    u32 ipc_shm_map_refs;
    u32 capsule_id;
    char image_path[96];
    u32 quota_mem_kib;
    u32 quota_cpu_ms;
    u32 quota_ipc_objs;
    u32 quota_fs_write_kib;
    u32 usage_ipc_objs;
    u64 usage_fs_write_bytes;
    u32 exec_image_size;
    u32 exec_hash_baseline;
    u32 exec_hash_last;
    u64 exec_hash_next_check_tick;
    u32 exec_hash_check_nonce;
};

/* PIKEE (Pi Kernel Execution Environment) API table passed to processes in x0 at entry.
 * This is the COMPLETE userland API surface.
 * All pointers passed by userland are bounds-checked. */
struct kernel_api {
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
    u64 (*runtime_ms)(void);
    u64 (*monotonic_ms)(void);
    u64 (*utc_ms)(void);
    i32 (*set_utc_ms)(u64 utc_ms);
    u64 (*rtc_ms)(void);
    i32 (*set_tz_offset_min)(i32 offset_min);
    i32 (*get_tz_offset_min)(void);
    i32 (*list_tz_offsets)(i32 *out_offsets, u32 max_entries);

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
    i32 (*page_open)(const char *path, u32 page_size, u32 flags);
    i32 (*page_read)(i32 page_id, u64 page_idx, void *out_page, u32 out_len);
    i32 (*page_write)(i32 page_id, u64 page_idx, const void *in_page, u32 in_len);
    i32 (*page_flush)(i32 page_id);
    i32 (*page_stat)(i32 page_id, struct paged_io_stat *out);
    i32 (*page_close)(i32 page_id);

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
    i32 (*lock_create)(void);           /* binary lock (mutex-like), returns id */
    i32 (*lock_acquire)(i32 id);        /* acquire lock */
    i32 (*lock_release)(i32 id);        /* release lock */

    /* ---- Local KV store (Picowal model) ---- */
    i32 (*kv_put)(u32 key, const void *data, u32 len);
    i32 (*kv_get)(u32 key, void *out, u32 out_len);
    i32 (*kv_del)(u32 key);
    i32 (*kv_list)(u16 card, u32 *out_keys, u32 max_keys);

    /* ---- App foundations: events, logs, service registry, hooks ---- */
    i32 (*event_emit)(u32 type, const void *data, u32 len);
    i32 (*event_next)(struct appf_event_record *out);
    i32 (*log_write)(u32 level, const char *msg, u32 len);
    i32 (*log_next)(struct appf_log_record *out);
    i32 (*svc_register)(const char *name, u32 kind, u32 endpoint, u32 flags);
    i32 (*svc_resolve)(const char *name, struct appf_service_record *out);
    i32 (*svc_list)(struct appf_service_record *out, u32 max_entries);
    i32 (*hook_bind)(u32 hook_type, const char *service_name);
    i32 (*hook_emit)(u32 hook_type, const void *data, u32 len);

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

    /* ---- Unified virtual device stream IPC (pipes) ---- */
    i32 (*pipe_create)(const char *path, u32 type, u32 depth, u32 flags, u32 frame_max);
    i32 (*pipe_open)(const char *path, u32 type);
    i32 (*pipe_close)(i32 pipe_id);
    i32 (*pipe_read)(i32 pipe_id, void *buf, u32 len);        /* stream-only */
    i32 (*pipe_write)(i32 pipe_id, const void *buf, u32 len); /* stream-only */
    i32 (*pipe_send)(i32 pipe_id, const void *msg, u32 len);  /* slot-only */
    i32 (*pipe_recv)(i32 pipe_id, void *msg, u32 len);        /* slot-only */
    i32 (*pipe_stat)(i32 pipe_id, struct pipe_stat *out);

    /* ---- Kernel-enforced IPC channels + shared regions ---- */
    i32 (*ipc_fifo_create)(const char *name, u32 peer_principal, u32 owner_acl,
                           u32 peer_acl, u32 depth, u32 msg_max);
    i32 (*ipc_fifo_open)(const char *name, u32 want_acl);
    i32 (*ipc_fifo_send)(i32 channel_id, const void *data, u32 len);
    i32 (*ipc_fifo_recv)(i32 channel_id, void *out, u32 out_max); /* returns msg len */
    i32 (*ipc_shm_create)(const char *name, u32 peer_principal, u32 owner_acl,
                          u32 peer_acl, u32 size);
    i32 (*ipc_shm_open)(const char *name, u32 want_acl);
    i32 (*ipc_shm_map)(i32 region_id, u32 flags, void **addr_out, u32 *size_out);
    i32 (*ipc_shm_unmap)(i32 map_handle);

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
    i32 (*tensor_mul)(void *c, const void *a, const void *b);
    i32 (*tensor_scale)(void *b, const void *a, float scalar);
    i32 (*tensor_bind_kernel_blob)(u32 kernel_id, const void *uniform_data, u32 uniform_bytes,
                                   const u64 *shader_code, u32 shader_insts);
};

typedef struct kernel_api pikee_api;

/* Assembly context switch: save callee-saved regs to old, restore from new */
extern void ctx_switch(struct proc_context *old, struct proc_context *new_ctx);

void proc_init(void);              /* init scheduler on this core */
i32  proc_exec(const char *path);  /* load + start process from WALFS */
void proc_yield(void);             /* cooperative context switch */
NORETURN void proc_exit(u32 code); /* terminate current process */
void proc_schedule(void);          /* run scheduler loop (called from coreN_main) */
u32  proc_count(void);             /* number of active processes on this core */
bool proc_handle_fault(u64 esr, u64 elr, u64 far); /* kill faulting user proc */

/* Preemption (user cores only) */
#define PROC_PREEMPT_TIMER_HZ    1000U
#define PROC_PREEMPT_QUANTUM_MS  5U
void proc_preempt_init(u32 timer_hz, u32 quantum_ms);
void proc_irq_maybe_preempt(struct irq_frame *frame);
u64  proc_preemptions(void);
u32  proc_snapshot(struct proc_ui_entry *out, u32 max_entries);
u32  proc_capsule_snapshot(struct proc_capsule_ui_entry *out, u32 max_entries);
bool proc_kill_pid(u32 pid, u32 code);
i32  proc_launch_on_core(u32 target_core, const char *path);
i32  proc_launch_on_core_as(u32 target_core, const char *path, u32 principal_id);
i32  proc_launch_on_core_as_prio(u32 target_core, const char *path, u32 principal_id, u32 priority_class);
bool proc_set_priority(u32 pid, u32 priority_class);
bool proc_set_affinity(u32 pid, u32 core);
