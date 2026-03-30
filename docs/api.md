# Kernel API Surface

This document describes the current kernel API surface in two layers:

- Userland PIKEE API (`struct kernel_api` in `include/proc.h`)
- Kernel subsystem C APIs (`include/*.h`) used by core runtime/components

## 1) Userland PIKEE (Pi Kernel Execution Environment) API table

Authoritative definition: `include/proc.h` (`struct kernel_api`).

### Exact C signatures

```c
struct paged_io_stat {
    u64 inode_id;
    u64 file_size;
    u32 page_size;
    u32 flags;
} PACKED;

struct kernel_api {
    i32 (*yield)(void);
    i32 (*exit)(u32 code);
    u32 (*getpid)(void);

    void (*print)(const char *msg);
    void (*putc)(char c);
    i32  (*getc)(void);
    i32  (*try_getc)(void);

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

    i32 (*open)(const char *path, u32 flags);
    i32 (*creat)(const char *path, u32 flags, u32 mode);
    i32 (*read)(i32 fd, void *buf, u32 len);
    i32 (*write)(i32 fd, const void *buf, u32 len);
    i32 (*pread)(i32 fd, void *buf, u32 len, u64 offset);
    i32 (*pwrite)(i32 fd, const void *buf, u32 len, u64 offset);
    i32 (*close)(i32 fd);
    i32 (*stat)(const char *path, void *out);
    i32 (*mkdir)(const char *path);
    i32 (*unlink)(const char *path);
    i32 (*readdir)(const char *path, void *entries, u32 max_entries);
    i32 (*page_open)(const char *path, u32 page_size, u32 flags);
    i32 (*page_read)(i32 page_id, u64 page_idx, void *out_page, u32 out_len);
    i32 (*page_write)(i32 page_id, u64 page_idx, const void *in_page, u32 in_len);
    i32 (*page_flush)(i32 page_id);
    i32 (*page_stat)(i32 page_id, struct paged_io_stat *out);
    i32 (*page_close)(i32 page_id);

    void (*fb_putc)(char c);
    void (*fb_print)(const char *s);
    void (*fb_color)(u32 fg, u32 bg);
    void (*fb_clear)(u32 color);
    void (*fb_pixel)(u32 x, u32 y, u32 color);

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

    i32 (*resolve)(const char *hostname, u32 *ip_out);

    u32 (*whoami)(void);
    i32 (*auth)(const char *user, const char *pass);

    void *(*sbrk)(i32 increment);
    void *(*memset)(void *dst, i32 c, u32 n);
    void *(*memcpy)(void *dst, const void *src, u32 n);
    u32   (*strlen)(const char *s);

    i32 (*spawn)(const char *path);
    i32 (*wait)(i32 pid);
    u32 (*nprocs)(void);

    i32 (*sem_create)(u32 initial);
    i32 (*sem_wait)(i32 id);
    i32 (*sem_post)(i32 id);
    i32 (*lock_create)(void);
    i32 (*lock_acquire)(i32 id);
    i32 (*lock_release)(i32 id);
    i32 (*kv_put)(u32 key, const void *data, u32 len);
    i32 (*kv_get)(u32 key, void *out, u32 out_len);
    i32 (*kv_del)(u32 key);
    i32 (*kv_list)(u16 card, u32 *out_keys, u32 max_keys);

    i32 (*queue_create)(const char *name, u32 depth, u32 flags, u32 frame_max);
    i32 (*queue_push)(i32 qid, const void *data, u32 len);
    i32 (*queue_pop)(i32 qid, void *out, u32 out_max);
    i32 (*queue_len)(i32 qid);
    i32 (*stack_create)(const char *name, u32 depth, u32 flags, u32 frame_max);
    i32 (*stack_push)(i32 sid, const void *data, u32 len);
    i32 (*stack_pop)(i32 sid, void *out, u32 out_max);
    i32 (*stack_len)(i32 sid);
    i32 (*topic_create)(const char *name, u32 replay_window, u32 flags, u32 event_max);
    i32 (*topic_publish)(i32 tid, const void *data, u32 len);
    i32 (*topic_subscribe)(i32 tid);
    i32 (*topic_read)(i32 sub_id, void *out, u32 out_max);

    i32 (*pipe_create)(const char *path, u32 type, u32 depth, u32 flags, u32 frame_max);
    i32 (*pipe_open)(const char *path, u32 type);
    i32 (*pipe_close)(i32 pipe_id);
    i32 (*pipe_read)(i32 pipe_id, void *buf, u32 len);
    i32 (*pipe_write)(i32 pipe_id, const void *buf, u32 len);
    i32 (*pipe_send)(i32 pipe_id, const void *msg, u32 len);
    i32 (*pipe_recv)(i32 pipe_id, void *msg, u32 len);
    i32 (*pipe_stat)(i32 pipe_id, struct pipe_stat *out);

    i32 (*ipc_fifo_create)(const char *name, u32 peer_principal, u32 owner_acl,
                           u32 peer_acl, u32 depth, u32 msg_max);
    i32 (*ipc_fifo_open)(const char *name, u32 want_acl);
    i32 (*ipc_fifo_send)(i32 channel_id, const void *data, u32 len);
    i32 (*ipc_fifo_recv)(i32 channel_id, void *out, u32 out_max);
    i32 (*ipc_shm_create)(const char *name, u32 peer_principal, u32 owner_acl,
                          u32 peer_acl, u32 size);
    i32 (*ipc_shm_open)(const char *name, u32 want_acl);
    i32 (*ipc_shm_map)(i32 region_id, u32 flags, void **addr_out, u32 *size_out);
    i32 (*ipc_shm_unmap)(i32 map_handle);

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
```

### Process control

- `yield()`
- `exit(code)`
- `getpid()`

### Console

- `print(msg)`
- `putc(c)`
- `getc()`
- `try_getc()`

### Timer

- `ticks()`
- `sleep_ms(ms)`
- `sleep_us(us)`
- `runtime_ms()`
- `monotonic_ms()`
- `utc_ms()`
- `set_utc_ms(utc_ms)` *(admin-gated)*
- `rtc_ms()`
- `set_tz_offset_min(offset_min)` *(admin-gated, minutes from UTC)*
- `get_tz_offset_min()`
- `list_tz_offsets(out_offsets, max_entries)`

### Filesystem (WALFS via Core 1 IPC)

- `open(path, flags)`
- `creat(path, flags, mode)`
- `read(fd, buf, len)`
- `write(fd, buf, len)`
- `pread(fd, buf, len, offset)`
- `pwrite(fd, buf, len, offset)`
- `close(fd)`
- `stat(path, out)`
- `mkdir(path)`
- `unlink(path)`
- `readdir(path, entries, max_entries)`

Storage hardening baseline:

- Boot now performs `walfs_verify` and halts (PiSOD) on WAL metadata/record-chain corruption.
- Operator tooling now exposes `disk verify` to run the same integrity scan on demand.

Update/rollback baseline:

- Boot update state is persisted in Picowal (`deck0/record12`) with HMAC authentication.
- Console command `update stage <slot> [tries]` arms a staged A/B candidate with bounded boot attempts.
- If staged update is not confirmed (`update success`) before attempts are exhausted, boot state auto-rolls back to the previous slot.

### Paged I/O primitive

- `page_open(path, page_size, flags)`
- `page_read(page_id, page_idx, out_page, out_len)`
- `page_write(page_id, page_idx, in_page, in_len)`
- `page_flush(page_id)`
- `page_stat(page_id, out)`
- `page_close(page_id)`

### Framebuffer

- `fb_putc(c)`
- `fb_print(s)`
- `fb_color(fg, bg)`
- `fb_clear(color)`
- `fb_pixel(x, y, color)`

### Networking + DNS

- `socket(type)`
- `bind(fd, ip, port)`
- `connect(fd, ip, port)`
- `listen(fd, backlog)`
- `accept(fd, client_ip, client_port)`
- `send(fd, data, len)`
- `recv(fd, buf, len)`
- `sendto(fd, data, len, ip, port)`
- `recvfrom(fd, buf, len, src_ip, src_port)`
- `sock_close(fd)`
- `resolve(hostname, ip_out)`

### Identity

- `whoami()`
- `auth(user, pass)`

### Memory/libc helpers

- `sbrk(increment)`
- `memset(dst, c, n)`
- `memcpy(dst, src, n)`
- `strlen(s)`

### Process management

- `spawn(path)`
- `wait(pid)`
- `nprocs()`

### Capsule manifest enforcement (phase 1)

When a process is launched from `path`, the kernel also looks for `path.cap`.

If present, the manifest enables capsule policy enforcement for that process:

- `capsule=on|off` (default `on` when manifest exists)
- `group=<name>` (optional shared capsule identity across multiple executables)
- `vfs=<absolute_prefix>` (optional virtual fs root; fs paths resolve under this prefix)
- `mem_kib=<n>` (optional heap growth ceiling via `sbrk`; `0` means unlimited)
- `cpu_ms=<n>` (optional runtime-tick budget; process is terminated when exceeded)
- `ipc_objs=<n>` (optional max count of created IPC objects per process)
- `fs_write_kib=<n>` (optional max cumulative bytes written via `write`; `0` means unlimited)
- `spawn=allow|deny`
- `wait=allow|deny`
- `nprocs=allow|deny`
- `fs=<prefix1>,<prefix2>,...` (path-prefix allowlist for fs path APIs)
- `ipc=<prefix1>,<prefix2>,...` (name-prefix allowlist for queue/stack/topic + proc IPC names)
- `pipe=<prefix1>,<prefix2>,...` (path-prefix allowlist for pipe create/open)
- `cards=<n>|<a-b>,...` (Picowal card allowlist)
- `ports=<n>|<a-b>,...` (network port allowlist for bind/connect/sendto)

Manifest parsing is strict: unknown keys, malformed `key=value` lines, invalid numeric values, non-absolute path prefixes (`fs`/`pipe`/`vfs`), or path-escape segments (`..`) cause launch rejection.

Capsule-local shared memory is available via existing `ipc_shm_create/open/map` using names allowed by the capsule IPC policy.

Current syscall enforcement:

- FS path checks: `open/stat/mkdir/unlink/creat/readdir/page_open`
- Process visibility checks: `spawn/wait/nprocs`
- KV card checks: `kv_put/kv_get/kv_del/kv_list`
- IPC namespace checks: queue/stack/topic create, pipe create/open, `ipc_fifo_*`/`ipc_shm_*` create/open
- Network policy checks: `bind/connect/sendto` port allowlist + EL2 port ownership mapping

### Capsule/MMU hardening (phase 2)

- User tables no longer include broad shared DMA windows by default.
- IPC SHM window is now mapped on demand per-process after successful `ipc_shm_map`.
- When the process releases its final SHM map handle via `ipc_shm_unmap`, the SHM window is removed from that process table.
- `ipc_shm_create` executes with a temporary kernel table switch for pool initialization, then returns to the process table.

Note: this is currently coarse at 2MB block granularity (ARM L2 block map), so IPC SHM isolation is tightened but not yet per-4KB page.

### Capsule namespace scoping (phase 3)

- App foundation rings are capsule-scoped at read time:
  - `event_next` only delivers events from the caller's capsule namespace.
  - `log_next` only delivers logs from the caller's capsule namespace.
- Service registry visibility is capsule-scoped:
  - `svc_resolve` / `svc_list` only return services in the caller's capsule namespace.
  - `svc_register` cannot overwrite entries outside the caller's capsule namespace.
- Hook bindings are capsule-scoped:
  - `hook_bind` updates only in-namespace bindings (unless admin + in-namespace ownership checks pass).
  - `hook_emit` dispatches only in-namespace bindings.
- In-memory IPC object handles are capsule-scoped in PIKEE wrappers:
  - queue/stack/topic create binds ownership namespace metadata.
  - queue/stack/topic operations reject out-of-namespace handles.

### EL2 stage-2 context selection (phase 4 follow-on)

- Process launch now binds capsule manifests to EL2 capsule slots when possible.
- Scheduler transitions select capsule stage-2 context via `EL2_HVC_STAGE2_ACTIVATE`:
  - process-in: selects that capsule's planned VTTBR/VTCR context.
  - process-out: deactivates to host/default context.
- Default app capsules are now bound with stage-2 VM activation intent, so user applications (including console apps) run capsule-first by default.
- Privileged device operations remain mediated by EL1 capability checks (principal/capability gates), not by bypassing capsule mode.
- On EL2 sync faults during stage-2 operation, EL2 records telemetry and automatically deactivates/disabled the active capsule's stage-2 context for fail-closed recovery.

### Runtime integrity attestation

- On process launch, the kernel computes and stores an executable baseline hash over the loaded image bytes.
- Scheduler-triggered periodic checks execute in EL2 (`EL2_HVC_INTEGRITY_CHECK`).
- If an EL0 executable hash drifts, **all processes in that capsule namespace are terminated**.
- EL2 validates both EL2 code integrity and boot-pinned EL1 text integrity during these checks.
- If EL1 or EL2 integrity drifts, kernel enters PiSOD halt.
- Kernel observability now exposes a consolidated `obs` console command showing:
  - process integrity check/failure counters and capsule-kill count
  - capsule firewall deny counters (manifest policy + EL2 port claim denies)
  - network drop totals and EL2 stage-2 fault telemetry

### Semaphores

- `sem_create(initial)`
- `sem_wait(id)`
- `sem_post(id)`

### Locks

- `lock_create()`
- `lock_acquire(id)`
- `lock_release(id)`

### Local KV store

- `kv_put(key, data, len)`
- `kv_get(key, out, out_len)`
- `kv_del(key)`
- `kv_list(card, out_keys, max_keys)`

### In-memory IPC

- `queue_create(name, depth, flags, frame_max)`
- `queue_push(qid, data, len)`
- `queue_pop(qid, out, out_max)`
- `queue_len(qid)`
- `stack_create(name, depth, flags, frame_max)`
- `stack_push(sid, data, len)`
- `stack_pop(sid, out, out_max)`
- `stack_len(sid)`
- `topic_create(name, replay_window, flags, event_max)`
- `topic_publish(tid, data, len)`
- `topic_subscribe(tid)`
- `topic_read(sub_id, out, out_max)`

### Unified virtual pipes

- `pipe_create(path, type, depth, flags, frame_max)`
- `pipe_open(path, type)`
- `pipe_close(pipe_id)`
- `pipe_read(pipe_id, buf, len)`
- `pipe_write(pipe_id, buf, len)`
- `pipe_send(pipe_id, msg, len)`
- `pipe_recv(pipe_id, msg, len)`
- `pipe_stat(pipe_id, out)`

### Kernel-enforced IPC channels/shared regions

- `ipc_fifo_create(name, peer_principal, owner_acl, peer_acl, depth, msg_max)`
- `ipc_fifo_open(name, want_acl)`
- `ipc_fifo_send(channel_id, data, len)`
- `ipc_fifo_recv(channel_id, out, out_max)`
- `ipc_shm_create(name, peer_principal, owner_acl, peer_acl, size)`
- `ipc_shm_open(name, want_acl)`
- `ipc_shm_map(region_id, flags, addr_out, size_out)`
- `ipc_shm_unmap(map_handle)`

### Tensor / GPU compute

- `tensor_alloc(t, rows, cols, elem_size)`
- `tensor_free(t)`
- `tensor_upload(t, data)`
- `tensor_download(t, data)`
- `tensor_matmul(c, a, b)`
- `tensor_relu(b, a)`
- `tensor_softmax(b, a)`
- `tensor_add(c, a, b)`
- `tensor_dot(result, a, b)`
- `tensor_mul(c, a, b)`  *(newly userland-exposed)*
- `tensor_scale(b, a, scalar)`  *(newly userland-exposed)*
- `tensor_bind_kernel_blob(kernel_id, uniform_data, uniform_bytes, shader_code, shader_insts)`  *(newly userland-exposed, admin-gated)*

## 2) Kernel subsystem APIs (C headers)

Primary kernel module APIs are declared in `include/*.h`:

- Core runtime: `core.h`, `setup.h`, `exception.h`, `timer.h`, `workq.h`, `ksem.h`
- Memory/MMU: `mmu.h`, `mmio.h`, `dma.h`
- Storage/FS: `sd.h`, `walfs.h`, `bcache.h`, `picowal_db.h`
- Networking: `net.h`, `arp.h`, `tcp.h`, `socket.h`, `dns.h`, `dhcp.h`, `tls.h`, `genet.h`
- GPU/compute: `gpu.h`, `v3d.h`, `tensor.h`
- IPC/process/user runtime: `fifo.h`, `ipc_queue.h`, `ipc_stream.h`, `ipc_proc.h`, `pipe.h`, `proc.h`, `principal.h`
- Platform/peripherals: `gic.h`, `uart.h`, `fb.h`, `pcie.h`, `rp1*.h`, `usb*.h`, `xhci.h`, `mailbox.h`
- SIMD/crypto helpers: `simd.h`, `crypto.h`

For exact signatures, use the corresponding header as source of truth.

## 3) Userland vs kernel-only notes

- If an operation is in `struct kernel_api`, it is userland-callable.
- Raw driver entrypoints (`genet_*`, `sd_*`, `v3d_*`, etc.) are kernel C APIs unless explicitly bridged by a PIKEE/IPC surface.
- `tensor_bind_kernel_blob` is now bridged to userland for controlled real-kernel bring-up (admin capability required).
