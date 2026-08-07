# PIOS Network Architecture and Operations

This is the authoritative map for the current PIOS network stack, NIC
backends, WiFi bring-up, counters, dashboard meters, and operator commands.
Older `networking.md` and `wifi.md` describe earlier implementations and should
not be used as the source of truth.

Active Pi 5 WPA association diagnosis is tracked in
[GitHub issue #76](https://github.com/WillEastbury/pios/issues/76).

PIOS is bare metal. There is no Linux network stack, libc socket layer, kernel
thread per connection, DHCP client, or userspace `wpa_supplicant`. Core 0 owns
the hardware network path and protocol state. Other cores communicate with
network services through bounded FIFO and capsule bridge protocols.

## Layering

```text
Application and service layer
  HTTP :80, HTTPS :443, debug :2323, admin/OTA :8080-:8082
  uhttp bridges :82/:83, capsvc-registered services (for example :8090)
  TLS 1.3 termination and capsule routing
                    |
Transport layer     |  tcp.c, udp callbacks
  TCP, UDP          |
                    |
Network layer       |  net.c, dns.c, arp.c
  IPv4, ICMP, DNS, routes, ARP/neighbor resolution
                    |
NIC boundary        |  nic.c
  validation, firewall, packet classification, counters, offload policy
                    |
Backend abstraction |  struct nic_ops
  Pi 5 wired: macb.c/macb_rx_engine.c (Cadence GEM)
  QEMU:       virtio_net.c
  Pi 5 WiFi: wifi_nic.c -> cyw43.c -> sdio.c
                    |
Hardware
  RP1 Ethernet via PCIe
  BCM2712 SDIO2 -> CYW43455
  virtio-mmio in QEMU
```

The active IP stack sees one `nic_ops` backend at a time. WiFi initialization
is additive: loading CYW43455 firmware does not replace the wired backend.
`wifi activate` performs the explicit backend switch only after association.

## Core ownership and communication

### Core 0 reactor

Core 0 owns:

- NIC descriptor and SDIO polling.
- Ethernet validation and firewalling.
- ARP, routes, IPv4, ICMP, UDP, TCP, and DNS.
- TLS records and the in-kernel service dispatch boundary.
- Network diagnostics and performance snapshots.

The wired receive path is IRQ-driven with bounded polling and recovery:

```text
GEM/RP1 Ethernet IRQ
  -> core0_eth_irq_handler()
  -> set CORE0_IO_NET and CORE0_IO_TCP
  -> core 0 reactor drains net_poll()
  -> accepted TCP input schedules TCP/service work immediately
  -> admin, echo, TLS, uhttp bridge, and capsvc pumps run
```

Periodic reactor ticks remain a correctness backstop for TX pumping, TCP
timers, DNS, WiFi polling, and NIC liveness recovery.

### Other cores

User cores do not mutate NIC, route, TCP, or scheduler-owned network state
directly. Cross-core operations use:

- 64-byte SPSC FIFO messages.
- Targeted SGI notification with SEV as a sticky correctness backstop.
- Opaque connection/service identifiers rather than raw pointers.
- `uhttp_bridge` and `capsvc` request/response ownership zones for EL0
  services.

UDP requests from user cores are delivered to core 0 through FIFO messages.
TCP-terminated capsule services are fed by kernel-owned bridge descriptors;
capsules do not own TCP control blocks.

## Receive path

```text
backend recv
  -> nic_recv()
     - frame bounds
     - destination filtering
     - firewall rules
     - protocol counters
     - per-packet checksum trust
  -> net_poll()
     - EtherType dispatch
     - ARP handling
     - IPv4 version/IHL/length/checksum validation
     - reject options and fragments
     - destination and source validation
  -> ICMP, UDP, or TCP
  -> service/capsule dispatch
```

`net_poll()` drains a bounded burst. It never treats an empty ring as an error.
Malformed input is rejected before protocol side effects, and each major drop
reason has a counter.

The Pi 5 GEM receive engine lives in the Normal-NC DMA_NET arena. Its ring,
buffers, ownership words, and recovery telemetry must never be remapped WB or
aliased with conflicting attributes.

## Transmit path

```text
service or protocol output
  -> route lookup
  -> next-hop selection
  -> static neighbor or dynamic ARP resolution
  -> Ethernet/IP/TCP, UDP, or ICMP construction
  -> nic_send() / nic_send_parts()
  -> active backend
```

TCP can segment directly from its per-connection ring buffer and use
header/payload split TX. Checksum offload is used only when the active backend
reports the relevant capability and per-packet contract.

## Addressing and routes

Current Pi 5 development addresses:

| Interface | Address | Activation |
|---|---:|---|
| Wired | `192.168.0.201/16` | Boot-time management path |
| WiFi | `192.168.0.202/16` | Only after successful `wifi activate` |
| Gateway/DNS | `192.168.0.1` | Static configuration |

PIOS uses explicit static configuration. Dynamic ARP is implemented and
coexists with static neighbor entries. Route lookup chooses a connected route
or gateway, then resolves the next-hop MAC. Resolution failure is explicit;
send paths do not fabricate success.

`TCP_BUF_SIZE` is 8192. The earlier concurrent-QEMU failure was caused by the
extra static fallback TCB storage crossing the old QEMU core-0 RAM boundary,
not by virtio descriptor capacity. The relocated QEMU memory map and linker
assertion now leave 580 KiB of measured margin at this size.

## NIC backends

### Pi 5 wired Ethernet

Files:

- `src/macb.c`
- `src/macb_rx_engine.c`
- `src/rp1.c`
- `src/pcie.c`

The Cadence GEM MAC is reached through RP1 over PCIe. Wired Ethernet is the
preferred management path during hardware bring-up. The RX engine owns:

- Descriptor publication and cursor state.
- BNA/OVR/HRESP recovery.
- Ordered-ring hole detection and capture.
- Activity-gated liveness recovery.
- RX wedge, idle, and recovery counters.

The historical RX descriptor-hole root cause was a boot-time WB-to-NC
attribute transition. DMA_NET is now Normal-NC from the first MMU enable.

### QEMU virtio-net

`virtio_net.c` implements the same `nic_ops` surface. QEMU has no CYW43455 and
no V3D hardware; network regression uses the active virtio backend while
preserving the same IP/TCP/service layers.

### Pi 5 WiFi

```text
wifi_nic.c
  -> cyw43.c: scan/join, SDPCM, BCDC, events, Ethernet frames
  -> sdio.c: BCM2712 SDIO2 host, CMD52/CMD53
  -> CYW43455 firmware
```

Important hardware facts:

- Controller: BCM2712 SDIO2 at `0x1001100000`, not RP1.
- Function 1 block size: 64 bytes.
- Function 2 block size: 512 bytes.
- Firmware upload batches: mandatory 64 blocks / 4 KiB.
- Bus: 4-bit, 50 MHz high-speed when advertised.
- No single-block fallback is allowed for the verified firmware upload path.
- CYW RAM is discovered from the backplane; the live CYW43455 exposes
  `0xC8000` bytes.

SDPCM channels:

| Channel | Use |
|---:|---|
| 0 | BCDC control |
| 1 | Firmware events |
| 2 | Ethernet data and EAPOL |
| 3 | Glom |

The SDPCM header is 12 bytes. Publication and consumption include sequence,
credit, frame indication, `nextlen`, and bounded transfer contracts.

## Bring-up

### Wired boot path

1. Initialize PCIe and RP1.
2. Initialize GEM descriptor and buffer arenas.
3. Discover/configure the PHY.
4. Select the wired `nic_ops` backend.
5. Call `net_init()` with the wired static address, route, and DNS state.
6. Install service listeners and default firewall policy.
7. Enter the core 0 reactor.

If wired bring-up fails, diagnostics must report failure; WiFi is not silently
treated as an equivalent successful fallback during active Pi 5 WiFi
development.

### WiFi development path

Credentials belong only in ignored `tools/wifi_config.local.json`. Never print
or commit them.

```powershell
# Stream firmware, NVRAM, and CLM into the running board.
python tools\pios_wifi_upload.py --host 192.168.0.201 --dir wifi_fw_staging

# Then issue through /api/terminal or a console:
wifi init
wifi scan
wifi results
wifi status
wifi fwlog

# Join using the ignored local config and a host-derived PMK.
python tools\pios_wifi_join.py `
  --config tools\wifi_config.local.json `
  --timeout 75 `
  --no-scan
```

Do not request activation while diagnosing association:

```text
wifi activate
```

is the final explicit step, not part of firmware initialization or join.

### WiFi state separation

These states are intentionally separate:

1. SDIO controller initialized.
2. CYW chip/backplane discovered.
3. Firmware/NVRAM/CLM loaded.
4. Radio ready.
5. Scan running/results available.
6. 802.11 association in progress.
7. WPA authorization complete.
8. WiFi backend activated for the IP stack.

Failure at any earlier state must leave wired `.201` usable and WiFi `.202`
inactive.

## Dashboard meters

The workbench header exposes two independent network views.

### Address rows

- `WIRED IP` always shows the wired management address and prefix.
- `WIRELESS IP` shows the configured WiFi address and `up`/`down` from the
  CYW link state.

### `NET:` meter

`NET:` represents the currently active generic NIC backend:

```text
NET: Rx=<current>/<capacity> Tx=<current>/<capacity> Mb/s <link><FD|HD>
```

- Byte totals come from `nic_packet_counters()`.
- Rates are sampled from counter deltas at a minimum 250 ms interval.
- Values use Mbps x1000 internally.
- Capacity is the active backend link speed.
- Peak active-NIC rates are also exported through `/api/status`.

### `WIFI:` meter

`WIFI:` is independent of active-backend selection:

```text
WIFI: Rx=<current>/150 Tx=<current>/150 Mb/s up|down
```

- Byte totals come from `wifi_nic_counters()`.
- Counters increment only on successful CYW Ethernet frame RX/TX.
- Dashboard rates are calculated from the previous dashboard sample.
- The displayed 150 Mbps capacity is a current reviewed display constant, not
  negotiated PHY telemetry.
- WiFi counters reset when `wifi_nic_init()` starts a new initialization.

This separation lets operators see WiFi traffic during bring-up without
switching the system away from wired management.

## Status and counters

`GET /api/status` exposes active-NIC metrics under `perf`, including:

- `nic_rx_bytes`, `nic_tx_bytes`
- `nic_rx_mbps_x1000`, `nic_tx_mbps_x1000`
- `nic_rx_peak_mbps_x1000`, `nic_tx_peak_mbps_x1000`
- `nic_rx_wedge`, `nic_rx_hole_recover`, `nic_rx_idle`
- `nic_link_mbps`, `nic_link_full_duplex`
- `nic_rx_capacity_mbps`, `nic_tx_capacity_mbps`
- `net_listen_count`, `net_listen_pending`

The WiFi dashboard byte/rate counters are currently independent display
telemetry and are not yet exported in `/api/status`.

### Counter ownership

| Counter family | Owner | Meaning |
|---|---|---|
| NIC packet counters | `nic.c` | L2 RX/TX, protocol classification, firewall and checksum outcomes |
| Network stats | `net.c` | IPv4/ICMP/UDP validation, dispatch, drops, poll behavior |
| TCP diagnostics | `tcp.c` | listeners, states, SYN cookies, retries, active-connect failures |
| ARP diagnostics | `arp.c` | requests, replies, learning, spoof/rate-limit/conflict drops |
| GEM diagnostics | `macb.c`, `macb_rx_engine.c` | descriptors, DMA state, wedges, holes, recoveries, pause frames |
| WiFi byte counters | `wifi_nic.c` | successful CYW Ethernet RX/TX |
| SDIO/CYW diagnostics | `sdio.c`, `cyw43.c` | host stages, CMD53, firmware, BCDC, events, scan/join, EAPOL |

Counters are diagnostics, not synchronization. A counter changing does not
publish ownership or authorize another core to mutate state.

## Operator commands

Commands are available through HTTP `/api/terminal` and, unless shadowed by a
frontend-specific handler, the UART/TCP console.

### Topology and sessions

| Command | Purpose |
|---|---|
| `netstat` | TCP listeners/sessions, owners, pending/RX/TX/retry state and summary diagnostics |
| `services` | Listening ports mapped to kernel or capsule process/core/description |
| `arp` | ARP table |
| `route` | Route table |
| `dns status` | DNS resolver/cache state |
| `dns flush` | Clear DNS cache |
| `dnslookup <host>` | Resolve a hostname |
| `ping <ip-or-cached-host> [count]` | ICMP echo, count 1-20 |
| `traceroute <ip-or-cached-host> [max_hops]` | TTL probes, maximum 30 hops |

### NIC and protocol diagnostics

| Command | Purpose |
|---|---|
| `nic counters` | L2/protocol totals, drops, firewall, rate-limit and checksum counters |
| `nic offload` | Checksum/TSO capability, enablement, usage and MAC registers |
| `nic dump on|off` | Packet dump control; use only for bounded diagnostics |
| `rxdiag` | One-shot MAC, NIC, IP-poll, IRQ and recovery summary |
| `macbdiag` | Pi 5 GEM ring ownership, DMA status, hole/liveness and TX telemetry |
| `rxholedump` | Preserved ordered-ring hole capture, if one exists |
| `arp status` | ARP requests/replies, spoof/rate-limit/conflict diagnostics |
| `arp probe` | Send an explicit ARP probe |
| `cachestats` | WALFS, DNS and ARP cache hit/miss/eviction totals |

Firewall commands and detailed syntax are documented in `commands.md` and
`CONSOLE.md`.

### WiFi commands

| Command | Purpose |
|---|---|
| `wifi status` | SDIO, CYW firmware, BCDC, event, scan and EAPOL diagnostics |
| `wifi probe` | Guarded SDIO2 enumeration probe |
| `wifi prepare` | Preload blobs and initialize chip/backplane |
| `wifi load` | Load firmware into an already prepared chip |
| `wifi init` | Full explicit WiFi initialization |
| `wifi scan` | Start escan |
| `wifi results` | Show deduplicated SSID/BSSID/channel/RSSI/security results |
| `wifi joindiag` | Query join-related firmware state when responses are available |
| `wifi fwlog` | Bounded CYW firmware console-ring dump |
| `wifi joinpmk <ssid> <64-hex-pmk>` | WPA2 join using a derived PMK |
| `wifi join <ssid> <pass>` | Firmware passphrase path; avoid exposing credentials in history |
| `wifi join3 <ssid> <pass>` | Experimental SAE path; currently quarantined |
| `wifi activate` | Switch active NIC and initialize `.202`; only after proven association |
| `wifi disconnect` | Request disassociation |

Prefer `tools/pios_wifi_join.py` over typing secrets into a command surface.

## Diagnostic order

For an unreachable or slow board, diagnose bottom-up:

1. `GET /api/status`: version, uptime, `error`, active-NIC rates and wedge/hole
   counters.
2. `macbdiag` or platform backend diagnostics: descriptor/DMA progress.
3. `rxdiag`: correlate MAC, NIC, IP polling and IRQ fallback.
4. `nic counters`: firewall, malformed, protocol and checksum outcomes.
5. `arp status`, `arp`, `route`: next-hop reachability.
6. `netstat`, `services`: transport and owner state.
7. `dns status`, `ping`, `traceroute`: client-path verification.
8. WiFi only: `wifi status`, `wifi results`, `wifi fwlog`.

For WiFi, do not debug WPA before proving:

- SDIO function 1 and 2 are ready.
- HT clock and CLM completed.
- D11 is out of reset.
- Scan results are structurally valid.
- No repeated `WLC_UP`/`wl_open` loop is present.

## Correctness constraints

- No mutable shared control structure may cross a cache-line boundary.
- DMA and shared control mappings must retain identical attributes across all
  visible aliases.
- Descriptor and FIFO publication requires payload-before-sequence release
  ordering; consumption requires acquire ordering.
- SPSC rings remain SPSC.
- Core A changes core B's state through messages, not remote field writes.
- Length and capacity are checked before touching packet, descriptor, event,
  BCDC, or SDPCM payloads.
- Parser and polling loops are bounded.
- No allocation or dynamic formatting in IRQ/hot paths.
- Impossible descriptor, ownership, generation, attribute, or protocol states
  fail closed with preserved diagnostics.

## Related documents

- `architecture_system.md` - cores, scheduling/quanta, the software-interrupt
  and quantum model, FIFOs, IPC, EL levels and memory isolation.
- `network_stack.md` - NIC/IP/TCP/UDP implementation detail, the cross-core FIFO
  network path, and TLS 1.3 termination with capsule offload.
- `boot_storage.md` - two-stage boot, A/B OTA, WALFS, users, logging and
  perf/monitoring.
- `gotchas.md` - failed attempts, reverted changes and non-obvious traps.
- `architecture.md` - core assignment, memory map and boot architecture.
- `kernel_dma_irq.md` - DMA, IRQ and core-0 reactor behavior.
- `ipc.md` - FIFO ownership, publication and wake contracts.
- `net_capsule_fifo.md` - capsule service dispatch architecture.
- `commands.md` - complete command syntax.
- `drivers.md` - NIC, PCIe, RP1, SDIO and hardware drivers.
- `deployment.md` - board deployment and recovery.
