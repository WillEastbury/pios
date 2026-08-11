# PIOS Boot, Storage, Users and Monitoring

How PIOS boots and updates itself, how the disk is laid out, how WALFS works,
and how identity, logging and telemetry are implemented.

Checked against the code. Companion documents:
[`architecture_system.md`](architecture_system.md),
[`network_stack.md`](network_stack.md), [`network.md`](network.md),
[`gotchas.md`](gotchas.md).

---

## 1. Two-stage boot

### 1.1 Why two stages

The GPU firmware loads exactly one file, `kernel8.img`. If that were the whole
kernel, an OTA update would be a single non-atomic overwrite of the only
bootable image — one bad flash and the board needs physical access.

So `kernel8.img` is a **tiny stage0 loader** (~25 KB). The real kernel (~3.5 MB)
lives in redundant raw slots that stage0 selects between. A failed update costs
a rollback, not a card reader.

### 1.2 Stage0

`bootstrap_start.S` → stack, BSS clear, board detect, MMU tables, MMU/caches on
→ `bootstrap_main()` (`bootstrap.c`). Secondary cores park.

Load addresses are platform-conditional:

| Symbol | Pi 5 / real HW | QEMU virt |
|---|---|---|
| `BOOT_DST_ADDR` | `0x00080000` | `0x40080000` |
| `BOOT_STAGING_ADDR` | `0x08000000` | `0x48000000` |
| `BOOT_TRAMP_ADDR` | `0x07FFF000` | `0x47FFF000` |

QEMU `-M virt` has no RAM below `0x40000000`, hence the split.

Stage0 contains its **own** read-only FAT32 reader (separate from `src/fat32.c`):
read MBR sector 0, verify partition 1 is FAT32 (type `0x0B`/`0x0C`), read the
BPB, walk cluster chains.

### 1.3 Payload selection

1. If `PIOSSTG2.PKG` exists on the FAT partition, load it to
   `BOOT_STAGING_ADDR`, parse the manifest, select the entry matching this
   platform, write it into the raw slot and update boot control.
2. Otherwise boot from the raw slot: **pending → active → FAT fallback**
   (`BOOT_FALLBACK_LBA` 2048).

Platform id: QEMU builds always select `QEMU_VIRT`; otherwise
`BOARD_FAMILY_BCM2837 → BCM2837_FAMILY`, else `PI5`.

Stage0 **fails closed** on a packaged entry unless `load_addr == BOOT_DST_ADDR`,
`payload_size <= PIOS_STAGE2_ZONE_BYTES`, and the platform matches.

### 1.4 Package format

`tools/build_stage2_package.py` emits a `PGS2` container: magic `0x32534750`,
version 1, 32-byte header, 112-byte entries, `FLAG_PACKAGED = 1`, package id at
byte 16. Each entry carries platform, required/optional feature bitmasks,
payload offset/size, load address, memory size, flags, codec and uncompressed
size. Multiple platform payloads can share one package (`--pi`, `--bcm2837`,
`--qemu`), each aligned to 512 bytes.

Caps: `MAX_PAYLOAD` `0x37FE00` per raw slot; 16 MiB per package.

> The Pi 5 payload alone is ~3.2 MB (the embedded PicoScript IDE blob is
> ~1.9 MB), so a combined Pi5+QEMU package no longer fits one slot. The Pi 5
> FAT package is therefore built **Pi5-only**; QEMU has its own package.

### 1.5 The trampoline

`bootstrap_trampoline.S` copies the payload to `BOOT_DST_ADDR` — which is
stage0's *own link address*, so it overwrites itself — then jumps.

Ordering is platform-conditional:

- **Real hardware:** copy first, then disable MMU/caches.
- **QEMU virt:** disable MMU/caches **before** the copy.

See [`gotchas.md`](gotchas.md) for why the two differ.

### 1.6 Stage2

`start.S`: `_start` masks DAIF immediately (UEFI can enter EL1 with IRQs
enabled and pending firmware interrupts would storm `irq_handler` before
`gic_init()`), EL2→EL1, SCTLR, NEON, SP/SP_EL1, `VBAR_EL1`, BSS clear,
`kernel_main()`.

---

## 2. A/B slots and OTA

### 2.1 Layout constants

From `include/walfs.h`:

| Symbol | Value |
|---|---|
| `PIOS_STAGE2_OFFSET` | `0x200` |
| `PIOS_STAGE2_END_OFFSET` | `0x37FFFF` |
| `PIOS_STAGE2_ZONE_BYTES` | `0x37FE00` |
| Slot A offset | `0x000000` |
| Slot B offset | `0x400000` |
| Bootctrl offset | `0x380000` |
| Bootctrl magic / version | `'PBC0'` / 1 |
| Default tries | 1 |

Bootctrl fields: `active`, `pending`, `tries_left`, `last_boot`, `good_mask`,
`generation`, `checksum`. Its LBA is
`walfs_partition_lba() + PIOS_BOOTCTRL_OFFSET/512`.

### 2.2 State machine

- `pios_bootctrl_target_slot()` — the slot *opposite* `active`; defaults to B if
  the read fails.
- `pios_bootctrl_mark_pending()` — set `pending`, `tries = 1`, clear `last_boot`,
  clear that slot's good bit, bump generation.
- `pios_bootctrl_mark_success()` — `active = last_boot`, clear pending, clear
  tries, OR the booted slot into `good_mask`.

A pending slot that fails to boot exhausts its single try and stage0 falls back
to the previous active slot.

### 2.3 Endpoints

| Port | Purpose |
|---:|---|
| 8080 | status |
| 8081 | reboot |
| 8082 | update |
| 2323 | debug console (requires `unlock pios`) |

`/api/admin/kernel-update` (requires `confirm=1`) supports
`status`, `self`, `begin`, `chunk`/`data`, `commit`, `writeandreboot`, `cancel`,
`reset`. `/api/admin/kernel-stream` is a single-connection streaming POST.

### 2.4 The two upload paths

**Streaming** (default in `tools/pios_ota_update.py`): one connection, body
streamed into `ota_stage_buf`, flushed to the raw slot on completion. TCP's
receive window self-paces it — no connection churn, no RX-ring overrun.

**Chunked** (`--chunked`): `begin` → repeated `chunk` (offset-validated,
resumable) → `commit`. Slower but each chunk is individually acknowledged, so it
grinds through a NIC wedge and resumes from the server's acknowledged
`nextOffset`.

> Practical note: the resumable chunked path is the more robust of the two when
> the board is under stress, because a lost response costs one chunk rather than
> the whole transfer.

Staging is `ota_stage_buf` (highmem when available, capacity
`PIOS_STAGE2_ZONE_BYTES`; QEMU uses a static fallback). Commit order is
**payload first, header last**: `http_write_kernel_payload_range()` then
`http_write_kernel_slot_header(..., true)`, and only then
`pios_bootctrl_mark_pending()`. A partial or aborted stream leaves the slot
uncommitted.

The original push throughput was ~17 KB/s with a 4 KB receive window.
`TCP_BUF_SIZE` is now 8192.

> ADR-019 records the QEMU static-memory-layout fix required before the larger
> OTA receive window was safe.

---

## 3. Disk layout

```text
LBA 0            MBR
LBA 2048         Partition 1: FAT32, 64 MiB   -> kernel8.img, PIOSSTG2.PKG,
                                                 firmware blobs, /wifi/
Partition 2      Raw:
   +0x000000       stage2 slot A   (zone 0x37FE00)
   +0x380000       boot control
   +0x400000       stage2 slot B
   +10 MiB         WALFS region
```

`WALFS_BOOT_SLOT_LBAS` = 10 MiB / 512 = 20480, so `WALFS_BASE_LBA` = 2048 +
20480 = **22528**. `configure_walfs_region()` subtracts the reserved boot region
from the partition size; `discover_partition()` prefers MBR partition 2 and can
fall back to p1 or whole-disk.

---

## 4. WALFS

### 4.1 Design

Append-only, crash-safe write-ahead log. Superblock at block 0, then
variable-length CRC32C-checked records that are **never overwritten**. "Latest
record for a given inode wins."

Record types: `INODE` 1, `DATA` 2, `DIRENT` 3, `DELETE` 4, `TX_BEGIN` 5,
`TX_COMMIT` 6.

Superblock: magic, version, block_size, total_blocks, `wal_head`, `tree_root`,
`record_count`, crc32, label.

Limits: `WALFS_NAME_MAX` 127, `WALFS_DATA_MAX` 16384, block size 512.

### 4.2 Mount and verify

Mount scans from the start, CRC-checks each record and rebuilds the in-memory
tree. `walfs_verify()` checks the superblock CRC, walks the records, and reports
`super`, `wal_head`, `valid_records`, `crc_errors`, `header_errors`, `open_tx`,
`scan_end` — returning true only with zero errors.

`walfs_sync()` flushes the superblock. `src/bcache.c` provides block caching.

### 4.3 Layered stores

- **`picowal_db.c`** — maps `(card, record)` to `/var/picowal/<card>/r<record>.rec`
  with key packing `key = (card << 22) | record`. Terminal verbs: `list`, `save`,
  `add`, `update`, `copy`, `rename`, `editor`. `save` is the binary-safe
  unconditional write and is the generic upload channel for any card.
- **`capsule_store.c`** — packs/cards under `/var/capsules`. Card roles: `0`
  manifest, `1..1000` exec, `1001..10000` source, `10001+` bytecode, `20000+`
  IPC.

`src/fat32.c` is a separate read-only FAT32 (with LFN) used at runtime for
firmware blobs; `src/sd.c` is raw SDHCI block I/O, PIO only, 512-byte blocks,
with a QEMU virtio-blk fallback (which requires two virtio-mmio BLK devices —
see [`gotchas.md`](gotchas.md)).

---

## 5. Users and identity

### 5.1 Principals

Persisted at `/var/picowal/c1/r1.rec`. `PRINCIPAL_MAX` 16, `PRINCIPAL_ROOT` 0.
Capabilities: `PRINCIPAL_ADMIN`, `NET`, `DISK`, `EXEC`, `IPC`.

`principal_init()` seeds `root` with all capabilities if no table exists.
`principal_has_cap()` returns true for root, otherwise checks flags; admin
implies IPC. Current identity is held per-core in a 64-byte-per-core array.

### 5.2 STS

`sts.c` issues HS256 JWTs (`{"alg":"HS256","typ":"JWT"}`) with a fixed scope and
audience policy. `sts_login()` authenticates and issues; validation extracts a
scope mask.

Enforced behaviours (all covered by smoke tests): login is **TLS-only** and
returns 403 on `:80`; with no signing secret it **fails closed**
(`err=no_signing_secret`); wrong password, insufficient scope and non-admin
bearer on admin routes are all rejected; `sts users` supports cursor pagination.

### 5.3 Keys and certificates

- `keystore.c` — seals/loads a root secret, deriving a wrap key from device
  identity. On Pi 5 that identity is the board serial via VideoCore mailbox; on
  QEMU (no mailbox, `PIOS_MBOX_BASE = 0`) it uses the virtual disk's provisioned
  MBR identity. `crypto_random_*` stays fail-closed — the disk id is a stable
  public identity, not a CSPRNG.
- `x509.c` — self-signed Ed25519 and P-256 certificates, CSRs, and
  `x509_p256_private_scalar()` for TLS CertificateVerify.
- `acme.c` — derives its account key via
  `keystore_derive_secret("acme-account-key-v1", ...)`; HTTP-01 challenge state.

---

## 6. Logging and tracing

### 6.1 HTTP log ring

`http_log_ring`, `HTTP_LOG_RING_SIZE` 64, sequence-numbered with
`tick_ms = timer_monotonic_ms()`. Served by `/logs?since=`, `/api/logs` and
`/api/admin/log-stream`; the response tail carries `PIOS log-stream next=<seq>`
for incremental polling.

### 6.2 dtrace

Per-core rings with an active category mask. Categories: `TCP`, `MAC`, `FIFO`,
`SCHED`, `IRQ`, `OTA`, `REACTOR`. Controls: `dtrace_set_enabled`,
`dtrace_set_mask`, `dtrace_clear`, `dtrace_dump`, `dtrace_status`.

Crash paths must preserve enough ring history for deterministic replay:
inbound descriptor ids, wake sequences, handler ids, binding ids.

### 6.3 Coredump

`coredump.c` keeps `g_dumps[COREDUMP_SLOTS]` snapshots and can format and diff
them (`cdump`). `exception_crash_persist_sd()` writes a crash record to SD so a
fault survives reboot.

### 6.4 Consoles

The UART console, the TCP debug console (`:2323`, gated behind `unlock pios`)
and HTTP `/api/terminal?cmd=` all dispatch into **one** shared implementation,
`http_exec_terminal_command()`. That is deliberate: the two front-ends can never
drift, and a command added once appears everywhere.

Panic paths may not allocate, may not block, and must be deterministic.

---

## 7. Perf and monitoring

### 7.1 `/api/status`

Top level: `ok`, `version`, `build`, `uptime`, `ip`, `mode`.

`diag`: `event`, `route`, `error`, `conn`, `req`, `resp`, `off`, `body`, `clen`,
`sched_wake`, `sched_wfi`, `sched_busy_permille`, `sched_flags`.

`perf` includes:

- CPU: `cpu_total_permille`, `cpu0..3_permille`, `cpu_clock_mhz`
- Board: revision, model, memory, manufacturer, PCB codes
- VideoCore/display: V3D presence, cores/slices/QPUs, display backend, geometry,
  scanout, takeover/dlist status
- RAM: installed/visible/physical, highmem base/limit/used/free, pool and kernel
  totals
- NIC: `nic_rx_bytes`, `nic_tx_bytes`, rates and peaks (`*_mbps_x1000`),
  `nic_rx_wedge`, `nic_rx_hole_recover`, `nic_rx_idle`, link speed/duplex,
  capacities
- Storage: SD read/write bytes, last/peak rates; WALFS mounted, super/root ok,
  partition LBA, region/used/free bytes, record count
- Dashboard: `dash_snap_ticks`, `dash_render_ticks`, `fb_blit_ticks`

`cpu_clock_mhz` comes from `perf_cpu_clock_measure()` using the PMU cycle
counter (`PMCCNTR_EL0`), sampled at 1 Hz offset by 500 ms so it never lands on
the same tick as the framebuffer-heavy render.

### 7.2 Dashboard

`hdmi_dashboard_render()` throttles to ≥900 ms (just under the 1 Hz
`CORE0_IO_DASH` cadence, so sub-ms beat never skips a second). Panels:

- `PIOS WORKBENCH` — version, uptime, `WIRED IP` and `WIRELESS IP` rows
- `HARDWARE / CAPABILITIES`
- `PARALLEL / VECTOR ACCELERATION` — CPU FP32, CPU integer, native V3D 7.1 CSD
  readiness, verified PicoScript QPU ops, QPU VM arithmetic, batched QPU MatVec,
  Media state. ADR-027 runs the guarded proofs and representative profiles during
  boot; the commands remain available for explicit re-verification. CPU FP32/NEON
  and CPU integer are always enabled; the integer row combines NEON INT8
  dot/matvec with scalar/C I32 primitives.
- `NETWORK / PROCESS MAP` — listening ports mapped to owning pid/core/process
- `WARNINGS / ERRORS`
- Optional right column: `NIC / MAC RX-TX`, `DMA ENGINE`, `FIFO / LEASE ARENAS`

Meters: `NET:` reflects the **active** backend from `nic_packet_counters()`;
`WIFI:` is independent, from `wifi_nic_counters()`, so WiFi traffic is visible
during bring-up without switching the system off wired management. Rates are
sampled from counter deltas at ≥250 ms.

Diagnostics must not perturb scheduling or change scheduler cache-line
ownership.

### 7.3 Service registry

`ksvc` is `services[NUM_CORES][KSVC_MAX_SERVICES]`, cache-line-sized entries,
per-core registration. Each entry tracks name, kind, owner core, priority,
state, generation, mailbox, calls, errors, total/last/max duration ticks,
messages sent/received, mailbox drops and restarts.

Registered today — core 0: `net-poll`, `tcp-http-tls`, `debug-console`,
`ui-input`, `dashboard`, `tcp-timers`, `sched-core0`, `fifo-core0`; cores 1–3:
`sched-coreN`, `fifo-coreN`.

**Counters are diagnostics, not synchronization.** A counter changing does not
publish ownership or authorize another core to mutate state.

### 7.4 Operator commands

`status`, `netstat`, `services`, `processes`/`ps`, `proc sched`, `rxdiag`,
`macbdiag`, `rxholedump`, `nic counters`, `nic offload`, `arp`, `route`,
`ping`, `traceroute`, `dnslookup`, `dns status`, `cachestats`, `walfs verify`,
`db …`, `capsule …`, `bootctrl status`, `selftest`, `dtrace …`, `cdump`,
`wifi …`, `tensor …`, `sts …`, `firewall …`.

### 7.5 Diagnostic order

For an unreachable or slow board, work bottom-up:

1. `/api/status` — version, uptime, `error`, rates, wedge/hole counters
2. `macbdiag` — descriptor/DMA progress
3. `rxdiag` — MAC vs NIC vs IP polling vs IRQ handoff
4. `nic counters` — firewall, malformed, protocol, checksum outcomes
5. `arp status`, `arp`, `route` — next-hop reachability
6. `netstat`, `services` — transport and ownership
7. `dns status`, `ping`, `traceroute` — client path
8. WiFi only: `wifi status`, `wifi results`, `wifi fwlog`

---

## 8. Build and deploy

```powershell
# Full Pi 5 build (stage0 + stage2 + QEMU payload + package)
cmd.exe /d /c "C:\source\pios\build_bootstrap.bat"

# Host-testable pure-logic suites
python tests\run_host_tests.py

# QEMU regression: 29 assertions + load battery
$env:PYTHONIOENCODING="utf-8"; python tools\qemu_smoke.py --build

# OTA to a live board (resumable path)
python tools\pios_ota_update.py real_kernel.img --host 192.168.0.201 `
    --chunked --reboot --commit-timeout 240 --timeout 15
```

Environment notes: do not assume `make` exists or that the AArch64 toolchain and
Visual Studio are on `PATH`. `build_bootstrap.bat` pins the toolchain to
`C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin`.

Verification standard: **zero errors and zero warnings** (`-Wall -Wextra` is
strict), host tests green, QEMU 29/29, then Pi 5 selftests 14/14 with
`error=0`, `rx_wedge=0`, `rx_hole_recover=0`.

Always leave the live board on a health-stable image.
