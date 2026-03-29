# Kernel API Surface

This document describes the current kernel API surface in two layers:

- Userland Kernel Program Interface (`struct kernel_api` in `include/proc.h`)
- Kernel subsystem C APIs (`include/*.h`) used by core runtime/components

## 1) Userland Kernel Program Interface (KPI table)

Authoritative definition: `include/proc.h` (`struct kernel_api`).

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

### Semaphores

- `sem_create(initial)`
- `sem_wait(id)`
- `sem_post(id)`

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
- Storage/FS: `sd.h`, `walfs.h`, `bcache.h`
- Networking: `net.h`, `arp.h`, `tcp.h`, `socket.h`, `dns.h`, `dhcp.h`, `tls.h`, `genet.h`
- GPU/compute: `gpu.h`, `v3d.h`, `tensor.h`
- IPC/process/user runtime: `fifo.h`, `ipc_queue.h`, `ipc_stream.h`, `ipc_proc.h`, `pipe.h`, `proc.h`, `principal.h`
- Platform/peripherals: `gic.h`, `uart.h`, `fb.h`, `pcie.h`, `rp1*.h`, `usb*.h`, `xhci.h`, `mailbox.h`
- SIMD/crypto helpers: `simd.h`, `crypto.h`

For exact signatures, use the corresponding header as source of truth.

## 3) Userland vs kernel-only notes

- If an operation is in `struct kernel_api`, it is userland-callable.
- Raw driver entrypoints (`genet_*`, `sd_*`, `v3d_*`, etc.) are kernel C APIs unless explicitly bridged by a KPI/IPC surface.
- `tensor_bind_kernel_blob` is now bridged to userland for controlled real-kernel bring-up (admin capability required).
