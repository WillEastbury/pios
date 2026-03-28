# Inter-Core Communication (IPC)

## Design

All inter-core communication uses **lock-free SPSC (Single-Producer Single-Consumer) ring buffers**. No locks, no atomics, no CAS. Just memory barriers (`dmb`) and event signalling (`sev`/`wfe`).

There are 16 FIFO channels arranged in a 4×4 grid (`fifo[src][dst]`). The diagonal (`fifo[i][i]`) is unused. This gives 12 active unidirectional channels.

## FIFO Location

FIFOs live in shared memory at `SHARED_FIFO_BASE` (0x04200000), outside any core's private RAM. Total: 1MB.

## Message Format

```c
struct fifo_msg {         // 64 bytes, cache-line aligned
    u32 type;             // message type (see below)
    u32 param;            // type-specific parameter
    u64 buffer;           // pointer to data (in sender's or shared RAM)
    u32 length;           // data length
    u32 status;           // response status (0 = ok)
    u64 tag;              // correlation tag (caller-defined)
    u64 timestamp;        // cntvct_el0 at send time
    u64 _reserved;
};
```

## Ring Structure

```c
struct fifo {
    volatile u32 head;    // written by producer ONLY (64-byte aligned)
    volatile u32 tail;    // written by consumer ONLY (64-byte aligned)
    struct fifo_msg msgs[512];
};
```

- **Capacity**: 512 messages (power-of-2 for bitmask indexing)
- **Full**: `(head + 1) & 511 == tail`
- **Empty**: `tail == head`
- **Push**: write message at `msgs[head]`, `dmb`, advance head, `sev`
- **Pop**: check `tail != head`, `dmb`, read `msgs[tail]`, `dmb`, advance tail

## Message Types

### Disk I/O (User → Core 1 → User)

| Type | Direction | Fields |
|------|-----------|--------|
| `MSG_DISK_READ` (1) | User → Core 1 | param=LBA, buffer=dest ptr, length=block count |
| `MSG_DISK_WRITE` (2) | User → Core 1 | param=LBA, buffer=src ptr, length=block count |
| `MSG_DISK_DONE` (3) | Core 1 → User | status=0, buffer=data ptr |
| `MSG_DISK_ERROR` (4) | Core 1 → User | status=error code |

### Network I/O (User → Core 0 → User)

| Type | Direction | Fields |
|------|-----------|--------|
| `MSG_NET_UDP_SEND` (10) | User → Core 0 | param=dst_ip, buffer=payload, length=len, tag=(src_port<<16)\|dst_port |
| `MSG_NET_UDP_RECV` (11) | Core 0 → User | param=src_ip, buffer=payload, length=len, tag=(src_port<<16)\|dst_port |
| `MSG_NET_UDP_DONE` (12) | Core 0 → User | status=0/1 |
| `MSG_NET_STATS` (13) | Either | buffer=pointer to net_stats_t |
| `MSG_NET_LINK_UP` (14) | Core 0 → User | Ethernet link up |
| `MSG_NET_LINK_DOWN` (15) | Core 0 → User | Ethernet link down |

### Generic

| Type | Direction | Fields |
|------|-----------|--------|
| `MSG_PING` (254) | Any → Any | Keepalive probe |
| `MSG_ACK` (255) | Any → Any | Acknowledgement |

## Usage Pattern

```c
// Core 2: Request a disk read
struct fifo_msg req = {
    .type   = MSG_DISK_READ,
    .param  = 100,                    // LBA
    .buffer = (u64)(usize)my_buffer,  // in core 2's 16MB
    .length = 1,
    .tag    = 42,                     // correlation ID
};
fifo_push(CORE_USER0, CORE_DISK, &req);

// Wait for reply
struct fifo_msg reply;
while (!fifo_pop(CORE_USER0, CORE_DISK, &reply))
    wfe();
// reply.type == MSG_DISK_DONE, reply.status == 0
```

## Buffer Ownership Rules

1. The `buffer` pointer in a request message points to memory **owned by the sender**
2. Core 1 (disk) copies data into/from the buffer, then sends a reply
3. The sender must not modify the buffer until it receives the reply
4. For large transfers, use memory in the shared DMA region (`DMA_DISK_BASE`)

## Performance

- Message push/pop: ~20ns (cache-line write + barrier + SEV)
- FIFO depth 512: can absorb bursts without backpressure
- `sev`/`wfe` wakes sleeping cores within ~100ns
