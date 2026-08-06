# First-Class Kernel Primitives Model

This is the target contract for PIOS as a primitives-first microkernel:

- Queue (`q`)
- Stack (`stack`)
- RAM region (`ram`)
- File (`file`)
- Paged I/O (`paged_io`)
- FIFO channel (`fifo`)
- Cache object (`cache`)
- TCP connection (`tcp_conn`)
- UDP endpoint (`udp_ep`)
- Security context (`sec_ctx`)

## Goals

- Each primitive is a kernel-owned object with stable handle semantics.
- Userland composes behavior from primitives instead of calling ad-hoc driver APIs.
- Security/capability policy is enforced at object boundary on every operation.
- All primitives expose explicit bounds, quotas, and deterministic failure modes.

## Uniform Object Contract

Every primitive follows the same lifecycle:

1. `create/open` (returns handle)
2. `read/write/send/recv/map` (type-specific ops)
3. `stat` (caps/owner/quota/state)
4. `close/destroy`

Common invariants:

- Handles are opaque kernel IDs (no user-controlled metadata writes).
- Every operation validates user pointers and principal capabilities.
- No unbounded internal allocations.
- Errors are explicit, never silent fallthrough.

## Primitive Definitions

## 1) Queue (`q`)

- Ordered bounded messages (FIFO discipline).
- Ops: `queue_create/push/pop/len`
- Policy: `PRINCIPAL_IPC` + object ACL.

## 2) Stack (`stack`)

- Ordered bounded messages (LIFO discipline).
- Ops: `stack_create/push/pop/len`
- Policy: `PRINCIPAL_IPC` + object ACL.

## 3) RAM Region (`ram`)

- Shared memory region with map/unmap handles.
- Backing: kernel-managed IPC SHM pool.
- Ops: `ipc_shm_create/open/map/unmap`
- Policy: `PRINCIPAL_IPC` + ACL flags.

## 4) File (`file`)

- WALFS inode-backed object.
- Ops: `open/creat/read/write/pread/pwrite/stat/mkdir/unlink/readdir`
- Policy: `PRINCIPAL_DISK` + inode ACL checks.

## 5) Paged I/O (`paged_io`)

- Explicit page/block object for fixed-size reads/writes.
- Ops:
  - `page_open(dev_or_file, page_size)`
  - `page_read(page_idx, out_page)`
  - `page_write(page_idx, in_page)`
  - `page_flush()`
  - `page_stat()`
  - `page_close()`
- Policy: disk capability + object ACL.
- Purpose: deterministic block/page semantics independent of stream/file APIs.
- Contract:
  - page size is power-of-two in `[512, 4096]`
  - reads/writes are positioned at `page_idx * page_size`
  - buffer length must equal object page size
  - handles are process-owned and non-transferable

## 6) FIFO Channel (`fifo`)

- Inter-core lock-free SPSC channels and process-level bounded FIFO channels.
- Ops:
  - Core-level: `fifo_push/pop/count`
  - Process-level: `ipc_fifo_create/open/send/recv`
- Policy: principal ACL (owner/peer), bounded depth/message size.

## 7) Cache Object (`cache`)

- Kernel-managed cache domains (bcache/fs/network state).
- Current: internal subsystem caches.
- Target userland surface:
  - `cache_stat(domain)`
  - `cache_hint(domain, key, policy)`
  - `cache_flush(domain, scope)`
- Policy: admin or domain capability.

## 8) TCP Connection (`tcp_conn`)

- Stateful stream object with kernel-managed sequence/retransmit windows.
- Userland via socket ABI:
  - `socket/bind/connect/listen/accept/send/recv/close`
- Policy: `PRINCIPAL_NET` + socket/object limits.

## 9) UDP Endpoint (`udp_ep`)

- Datagram endpoint object.
- Userland via socket ABI and net FIFO paths:
  - `socket/sendto/recvfrom`
  - `MSG_NET_UDP_SEND/RECV`
- Policy: `PRINCIPAL_NET`.

## 10) Security Context (`sec_ctx`)

- Principal identity + capability + ACL envelope attached to every object.
- Current primitives:
  - `whoami/auth`
  - capability checks in kernel API handlers
  - inode ACL checks
  - IPC owner/peer ACL checks
- Target extension: explicit `sec_ctx_stat(handle)` for any object.

## Mapping: Current vs Target

- Queue/Stack: **implemented**
- RAM (SHM): **implemented**
- File: **implemented**
- Paged I/O: **implemented**
- FIFO: **implemented** (core + process IPC)
- Cache: **partially implemented** (internal only)
- TCP/UDP: **implemented** (socket + net primitives)
- Security context: **implemented baseline**, object-introspection pending

## Layering Rule

- Kernel exports primitives only.
- Userland libraries build higher-level APIs by composing primitive handles.
- Driver-specific calls stay kernel-internal unless promoted to primitive form.

## Immediate Next Step

Extend paged I/O with explicit create/open mode flags and per-object ACL metadata export in `page_stat()`.
