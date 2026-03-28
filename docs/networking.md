# Networking

## Stack Design

Minimal, hardened, polling-based. Runs exclusively on Core 0.

```
Ethernet Frame (genet.c)
    │
    ▼
Frame Validation (min size, max size)
    │
    ▼
EtherType Dispatch
    ├── 0x0800 → IPv4
    │             ├── Version == 4?
    │             ├── IHL == 5? (reject options)
    │             ├── IP checksum valid? (NEON)
    │             ├── No fragments? (MF=0, offset=0)
    │             ├── Source IP valid?
    │             ├── Destination == our IP?
    │             │
    │             ├── ICMP (proto 1)
    │             │   └── Echo request → rate-limited reply
    │             │
    │             └── UDP (proto 17)
    │                 └── Validate length → callback
    │
    └── anything else → drop silently
```

## Security Model

The stack is **secure by omission**. Features that aren't implemented can't be attacked.

### What's Rejected (with per-reason drop counters)

| Check | Counter | What's Dropped |
|-------|---------|----------------|
| Frame < 14 bytes | `drop_runt` | Runt frames |
| Frame > 1518 bytes | `drop_oversized` | Oversized frames |
| IP checksum fail | `drop_bad_cksum` | Corrupted/crafted packets |
| IP fragments | `drop_fragment` | MF flag set or frag offset ≠ 0 |
| IP options (IHL > 5) | `drop_ip_options` | Source routing, record route, etc. |
| Bad source IP | `drop_bad_src` | 0.0.0.0, broadcast, self, loopback, multicast |
| Not our IP | `drop_not_for_us` | Wrong destination |
| Unknown protocol | `drop_bad_proto` | Anything not ICMP/UDP |
| ICMP too fast | `drop_icmp_ratelimit` | >10 pings/sec |
| UDP length mismatch | `drop_udp_malformed` | Header says more bytes than frame contains |
| No neighbor MAC | `drop_no_neighbor` | Can't resolve next-hop (static table miss) |

### Neighbor Resolution

**No ARP.** The neighbor table is static — entries are added via `net_add_neighbor(ip, mac)` at compile time or via FIFO command. The gateway MAC must be configured in `kernel.c` as `MY_GW_MAC`.

For destinations on the local subnet, the MAC must be in the static table. For off-subnet destinations, traffic goes via the gateway MAC.

## UDP Send Path

```
net_send_udp(dst_ip, src_port, dst_port, data, len)
    │
    ├── resolve_mac(dst_ip) → static neighbor lookup
    │   ├── on-subnet? → lookup dst_ip directly
    │   └── off-subnet? → use gateway MAC
    │
    ├── Build IP header (NEON checksum)
    │   ├── DF bit set (no fragmentation)
    │   ├── TTL = 64
    │   ├── ID from counter
    │   └── UDP checksum = 0 (optional in IPv4)
    │
    ├── Build Ethernet header
    └── genet_send(frame)
```

## FIFO Integration

User cores send/receive UDP through FIFO messages to Core 0:

```
User Core 2/3                         Core 0
    │                                    │
    ├─ MSG_NET_UDP_SEND ──────────────→ net_send_udp()
    │  .param   = dst_ip                │
    │  .buffer  = payload ptr           │
    │  .length  = payload len           │
    │  .tag     = (src_port<<16)|dst_port
    │                                    │
    ←── MSG_NET_UDP_DONE ───────────── reply
        .status = 0 (ok) or 1 (fail)
```

Incoming UDP is dispatched via `udp_callback` — currently only callable from Core 0 code. To route received UDP to user cores, register a callback that pushes `MSG_NET_UDP_RECV` into the appropriate FIFO.

## Configuration

In `kernel.c`:
```c
#define MY_IP       IP4(10, 0, 0, 2)
#define MY_GW       IP4(10, 0, 0, 1)
#define MY_MASK     IP4(255, 255, 255, 0)
static const u8 MY_GW_MAC[6] = { ... };
```

## Checksums

All IP checksums use `simd_checksum()` from `simd.c` — NEON pairwise reduction processing 32 bytes per iteration. This is ~10x faster than scalar for typical packet headers.
