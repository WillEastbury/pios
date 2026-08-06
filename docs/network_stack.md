# PIOS Network Stack Implementation

How packets actually move through PIOS: NIC backends, IP/ICMP/ARP/DNS, TCP, the
cross-core FIFO path for UDP and DNS, and TLS 1.3 termination with capsule
offload.

Checked against the code. Companion documents:
[`architecture_system.md`](architecture_system.md),
[`boot_storage.md`](boot_storage.md), [`network.md`](network.md) (operations),
[`gotchas.md`](gotchas.md) (failed attempts and traps).

---

## 1. Layering

```text
Services      HTTP :80, HTTPS :443, debug :2323, admin/OTA :8080-:8082
              uhttp bridges :82/:83, capsvc services (e.g. :8090)
                    |
Transport     tcp.c, UDP callbacks
                    |
Network       net.c, arp.c, dns.c   (IPv4, ICMP, routes, neighbours)
                    |
NIC boundary  nic.c   validation, firewall, classification, counters, offload
                    |
Backends      struct nic_ops
              Pi 5 wired: macb.c + macb_rx_engine.c   (Cadence GEM via RP1/PCIe)
              QEMU:       virtio_net.c
              Pi 5 WiFi:  wifi_nic.c -> cyw43.c -> sdio.c
                    |
Hardware      RP1 Ethernet (PCIe) | BCM2712 SDIO2 -> CYW43455 | virtio-mmio
```

Exactly **one** `nic_ops` backend is active at a time. WiFi initialisation is
additive: loading CYW43455 firmware does not replace the wired backend.
`wifi activate` performs the explicit switch, and only after association.

Core 0 owns all of this. Other cores reach it through FIFO messages and capsule
bridges, never by touching NIC, route or TCP state directly.

---

## 2. NIC boundary

`struct nic_ops` (`include/nic.h`) provides probe/init/send/recv/MAC/link plus
**optional** offload hooks, which may be `NULL`.

`nic_recv()` runs before any protocol handling:

- frame bounds check
- destination filtering
- firewall rules — `NIC_FILTER_MAX_RULES` 32, first-match-wins with a default
  policy; flags cover MAC, EtherType, IP protocol, IP ranges, TCP/UDP ports
- protocol classification and counters: `rx_arp`, `rx_ip`, `rx_tcp`, `rx_udp`,
  `firewalled`, `flood_blocked`, `rate_limited`, checksum trust/offload
- per-packet checksum trust

Checksum trust is **per backend and per packet**, never assumed. MACB derives it
from the descriptor checksum status and trusts it only when the checked mask
matches the expected L4 value. virtio-net and WiFi report
`checksum_trusted = false`.

`nic_init_wifi()` brings WiFi up; `nic_activate_wifi_loaded()` switches the
active backend only once WiFi is loaded and associated; `nic_active_name()`
reports the current backend or `"none"`.

---

## 3. Pi 5 wired: Cadence GEM

### 3.1 Ring layout

`NUM_RX` 896, `NUM_TX` 64, `BUF_SIZE` 2048. Rings **and** buffers live in the
dedicated Normal-NC `DMA_NET` arena (not `.bss`), alongside a dummy descriptor
for multi-queue init.

Descriptor ownership:

- RX word0 bit0 = `OWN` (0 = MAC owns, 1 = software owns), bit1 = `WRAP`
- TX `ctrl` bit31 = `USED` (1 = software owns), bit30 = `WRAP`

### 3.2 The RX engine

`macb_rx_engine.c` is a separate translation unit so it can be reasoned about
and fuzzed independently. After `macb_rx_engine_init()` it is the **exclusive
owner** of: RX `OWN` bits, the software cursor, `RBQP`/`RBQPH`, `NCR.RE` for RX
restart, and the `RSR` BNA/OVR/HRESP fault bits. The driver must not clear those
bits or rewrite the ring behind it.

Descriptor release reconstructs word 0 exactly rather than read-modify-write.

Three layered recoveries, all sharing one fail-closed rebuild primitive
(disable `NCR.RE`, *prove* it disabled, republish the whole ring while disabled,
reprogram `RBQP`/`RBQPH`, clear `RSR`, re-enable RE, reset the cursor):

| Recovery | Trigger |
|---|---|
| `recover_status()` | Latched BNA/OVR/HRESP |
| `recover_ordering_hole()` | Current descriptor unowned while ≥`RX_HOLE_MIN_LATER` (4) later ones are owned, stable ≥`RX_HOLE_HOLD_MS` (20 ms) |
| `recover_liveness()` | Proven RX progress, then `RX_LIVE_WEDGE_MS` (90 s) of RX silence while TX advances |

Every scan is bounded by `ring_count` or a fixed spin bound
(`RX_RE_DISABLE_SPINS` 100000).

### 3.3 Cacheability requirement

The ring, buffers, ownership words and recovery telemetry live in `DMA_NET`,
which is **Normal-NC from the first MMU enable** via a boot-only 2 MiB L2 split.
They must never be remapped WB or aliased with conflicting attributes.

This is a correctness requirement, not a tuning choice: a WB alias lets core 0
read a stale descriptor that GEM has already published, which presents exactly
like a DMA halt. See [`gotchas.md`](gotchas.md).

Proof of the current configuration: 12,898 concurrent `/picoscript` downloads,
3.14 GB in 300 s (83.77 Mbps), zero transfer errors, `rx_recv=3,081,956`,
`rx_recover=0`, `rx_hole_recover=0`, `rx_wedge=0`.

### 3.4 Known fragility (issue #82)

`macb.c` builds DMA addresses by hand at five sites: low word
`(u32)(usize)ptr`, high word hardcoded `MACB_DMA_HI` (`0x10`, the RP1 PCIe
inbound window). A buffer above 4 GiB would silently truncate. The fix is a
single `macb_dma_addr()` plus a **fail-closed** init check. `genet.c` already
does this correctly; `macb` is the outlier.

### 3.5 QEMU virtio-net

`VQ_SIZE` 32, `VNET_BUF_SIZE` 2048. Supports legacy single-region and modern
split queues, negotiates only offered features, prepends/strips the virtio-net
header (10 bytes legacy, 12 modern) and refills RX descriptors. Guest-owned
virtqueues — no `DMA_NET` ring and no GEM-specific recovery.

---

## 4. IP, ICMP, ARP, DNS

### 4.1 net.c

`net_poll()` drains a bounded burst — `NET_RX_BURST_MAX` 896 — and never treats
an empty ring as an error. Malformed input is rejected **before** protocol side
effects, and each major drop reason has a counter.

IPv4 validation rejects bad version/IHL/length/checksum, **options and
fragments**, and invalid source/destination. Routes use longest-prefix match;
next hop resolves through static neighbours or dynamic ARP. Resolution failure
is explicit — send paths never fabricate success.

Addressing is static by design (no DHCP client):

| Interface | Address | Activation |
|---|---|---|
| Wired | `192.168.0.201/16` | Boot-time management path |
| WiFi | `192.168.0.202/16` | Only after `wifi activate` |
| Gateway/DNS | `192.168.0.1` | Static |

ICMP is both responder (rate-limited) and client: `net_icmp_echo_send()` /
`net_icmp_echo_poll_result()` support `ping` and `traceroute`. Only one probe is
in flight at a time, which is why Time-Exceeded can be accepted unambiguously.
`net_icmp_echo_send_retry()` covers the cold-boot ARP window (see
[`gotchas.md`](gotchas.md)).

### 4.2 ARP

Reachable TTL 60 s, stale at 300 s, request interval 1 s/IP, **global** reply
rate limit 100 ms, 3 boot announcements, max 3 retries for incomplete entries.

A MAC change requires a consistency threshold of 2 replies before it is
accepted — anti-spoof. Conflict detection fires when `sender_ip == my_ip` with a
different MAC. Static and dynamic entries coexist.

### 4.3 DNS

Stub resolver: A records to port 53, randomised TXID and source port
(49152–65535), strict response validation, single-question enforcement, TTL
clamped to [60, 86400], 3000 ms timeout, max 3 retries, 60 s LRU cache.
`dns_resolve_async_start()` + `dns_poll()`; `dns_resolve()` is the synchronous
wrapper.

---

## 5. TCP

`TCP_BUF_SIZE` **4096**, `TCP_MSS` 1460, `TCP_MAX_CONNECTIONS` **128**,
`LISTEN_BACKLOG` 64.

> `TCP_BUF_SIZE` stays 4096; raising it has been tried and reverted. See
> [`gotchas.md`](gotchas.md).

`struct tcb` holds the 4-tuple, state, sequence state, TX/RX rings,
retransmission timers, RTT estimator, Reno congestion state, FIN/TIME_WAIT
state, listen backlog with pending SYN-cookie accepts, and intrusive hash/
free-list links. All 11 RFC 793 states are implemented.

- **SYN cookies** validate pending accepts, so a backlog flood cannot exhaust
  state.
- **Retransmission**: initial RTO 1000 ms, max 60000 ms, max 8 retries; Reno
  `cwnd`/`ssthresh` with fast retransmit on 3 duplicate ACKs.
- **Window self-heal**: `tcp_advertise_window()` re-sends the current ACK/window
  to recover from a lost window-update ACK, which otherwise stalls inbound bulk
  transfer near the end.
- **Segment directly from the ring**: `ring_linear_ptr_at()` /
  `tcp_send_segment_from_txbuf()` with header/payload split TX;
  `TCP_DMA_COPY_THRESHOLD` 256.

API: `tcp_connect`, `tcp_listen`, `tcp_accept`, `tcp_read`, `tcp_write`,
`tcp_close`, `tcp_abort`, `tcp_readable`, `tcp_writable`, `tcp_tx_pending`,
`tcp_snapshot`, `tcp_diag`, `tcp_table_stats` (surfaced by `netstat`).

---

## 6. UDP and the cross-core FIFO path

UDP dispatch has one active `udp_callback` plus a 4-slot subscriber list
(`UDP_SUBSCRIBER_MAX`). `handle_udp()` validates length and checksum, then calls
the callback and every subscriber.

User cores cannot touch the network stack directly. They post FIFO messages that
core 0 services in `net_handle_fifo_request()`:

| Request | Handling | Reply |
|---|---|---|
| `MSG_NET_UDP_SEND` | `net_send_udp(...)` | `MSG_NET_UDP_DONE` |
| `MSG_DNS_RESOLVE` | queued async via `dns_fifo_enqueue(...)` | `MSG_DNS_RESOLVE_DONE` |

Replies go back with `fifo_push(CORE_NET, requester_core, &reply)`, correlated
by `.tag`. `CORE_USER0` and `CORE_USER1` are handled separately.

Since the software-interrupt model landed, `fifo_notify()` also posts an
`AIRQ_SRC_FIFO_CORE(dst)` record at **HIGH** priority, so the target core's
dispatcher knows *what* is waiting rather than merely that it should wake. HIGH
exists precisely because a core is parked on a reply only the target can
produce — see [`architecture_system.md`](architecture_system.md) §3.

A DNS resolve from a user core carries a `status == 2` case meaning "core 0's
pending queue was full": the requester resends and keeps waiting rather than
failing a resolvable lookup.

---

## 7. TLS 1.3

### 7.1 What is implemented

`tls_accept()` is a real RFC 8446 server: **TLS_AES_128_GCM_SHA256**, **P-256
ECDHE**, **ECDSA-P256-SHA256** CertificateVerify. Flow: read ClientHello →
validate version/cipher/key share/signature scheme → derive ephemeral P-256 key
→ ServerHello → handshake traffic keys → EncryptedExtensions → Certificate →
CertificateVerify → Finished → application secrets.

The plaintext compatibility CCS record (`0x14`, payload `0x01`) is consumed and
ignored before client Finished, per RFC 8446 Appendix D.4 — up to 4 records.

`TLS_MAX_CONNECTIONS` connection pool; `TLS_IO_TIMEOUT` 5000 ms.

Identity comes from `x509_certificate_der()` + `x509_p256_private_scalar()`.

> `tls_connect()` (client) still uses the older non-standard CHLO/SHLO exchange.
> Only the **server** path is real TLS 1.3.

The reusable core is vendored from `picotlsserver`
(`picotlsserver.c`, `tls13_record.c`, `tls13_keysched.c`, `tls13_handshake.c`)
byte-identically; PIOS `tls.c` contains only the transport/random/x509/
connection-pool/diagnostic adapter. Verify with
`python C:\source\picotlsserver\tools\sync_pios.py --check`.

`crypto_selftest()` validates AES/GCM against FIPS-197 Appendix B/C and NIST
SP 800-38D Test Case 4 — independent published vectors, not self-consistency.
See [`gotchas.md`](gotchas.md) for why that matters.

---

## 8. Services and capsule offload

### 8.1 Port map

| Port | Served by |
|---:|---|
| 7 | kernel echo |
| 80 | kernel HTTP |
| 443 | kernel TLS 1.3 (+ `admin.pios` routed to a capsule) |
| 82, 83 | `uhttp_bridge` → EL0 PicoScript VM workers |
| 2323 | kernel debug console (`unlock pios`) |
| 8080/8081/8082 | admin status / reboot / update |
| 8090 | capsvc admin capsule |

### 8.2 What "offload" means here

Precisely: **the kernel terminates TCP and TLS; userland produces the response
body.** It is not encrypted pass-through.

```text
kernel accepts TCP (and terminates TLS on :443)
  -> copies the request into the shared arena (UHTTP_BRIDGE_ADDR == IPC_SHM_BASE)
  -> bumps req_seq, wakes the owning core (proc_post_remote_wake)
EL0 worker / capsule host
  -> pv_vm_run() executes PicoScript bytecode
  -> writes the reply into the arena, bumps resp_seq
kernel
  -> streams the reply over its own TCP connection
  -> closes only once tcp_tx_pending() == 0
```

The request/reply hot path costs **zero syscalls** — the arena is shared Normal-NC
memory; only `park()` is a syscall. Capsules never own TCP control blocks; the
kernel owns them and hands over bounded descriptors.

`capsvc_register(port, target_core)` registers a generic capsule service;
`capsvc_service_count()`/`capsvc_service_info()` enumerate them without capsvc
ever naming a specific service. That enumeration feeds the `services` command,
`netstat` owner labels and the dashboard's NETWORK/PROCESS MAP.

Programs load WALFS-first (matched by `io = tcp/<port>` + `entry="http"`),
falling back to a compiled-in default program when no card is installed.

---

## 9. Correctness constraints

- Ingress is validated and dropped fast; malformed input never reaches protocol
  side effects.
- Every drop reason has a counter; counters are diagnostics, not
  synchronization.
- DMA rings and buffers stay Normal-NC with identical attributes across every
  alias.
- Descriptor publication is release-ordered; consumption is acquire-ordered.
- SPSC rings stay SPSC.
- Cross-core changes are messages, never remote field writes.
- Parsing and polling loops are bounded; length is authority.
- No allocation and no dynamic string formatting in IRQ or hot paths.
- Failures are explicit: no success-shaped fallbacks, no silent single-block
  retries hiding a mandatory multi-block failure.
