# PIOS Command-Line Reference

Authoritative, surface-aware reference for every PIOS operator command. For
*how* to reach each surface and the broader operations workflow see
[../CONSOLE.md](../CONSOLE.md); for the kernel API/ABI see [api.md](api.md).

## Command surfaces

PIOS dispatches operator commands through **two** independent code paths:

| Surface | Transport | Dispatcher (`src/kernel.c`) | Output routing |
|---|---|---|---|
| **HTTP terminal** | `GET /api/terminal?cmd=<words+with+pluses>` (port 80) | `http_build_terminal_response()` | `http_append*` → HTTP body (clean text) |
| **UART / TCP console** | UART `115200 8N1`, HDMI mirror, or TCP `2323` after `unlock pios` | `ui_console_exec()` → `ui_cmd_*` | `ui_console_write()` → UART + HDMI + TCP-2323 |

Key differences:

- The **UART/TCP console is a superset**: it adds a shell (file ops, scripting,
  mounts, process launch) on top of the diagnostic command set.
- The **HTTP terminal is a bounded diagnostic subset** intended for the Web
  Admin console and remote scripting.
- Some `ui_cmd_*` handlers emit *detail* via `fb_printf` (HDMI-only); over
  TCP-2323 you see only what is written through `ui_console_write()`. Numeric
  output for TCP uses `ui_console_u32_dec` / `ui_console_u64_dec` /
  `ui_console_hex_fixed`.

In the tables below the **Surfaces** column marks availability:
`H` = HTTP terminal, `U` = UART/TCP console.

> **Spaces in HTTP commands** are encoded as `+`, e.g.
> `GET /api/terminal?cmd=walfs+verify`.

---

## Help

| Command | Surfaces | Description |
|---|---|---|
| `help` | H U | List commands / help topics. Category names are help topics, not prefixes. |
| `help <topic>` | H U | Per-command/category help. HTTP topics include `status netstat firewall reboot dma tls brotli walfs cachestats crypto watchdog arp nic …`. UART categories: `core fs net svc dev`. |

---

## Core & system

| Command | Surfaces | Description |
|---|---|---|
| `status` | H U | System/build/network summary (build label, uptime, RX/TX, drops). |
| `core status` / `sched` / `sched status` / `proc sched` | H U | PSCI/core stage plus scheduler busy, park/wake, soft-event, and boost counters. |
| `sgi stat` / `sgi test <core> [count]` | H U | SGI receive counts plus FIFO IRQ readiness/sent counters; bounded delivery probe. |
| `ps` / `processes` | H U | Process snapshot: PID/PPID, state, core, arena/span telemetry, graph. |
| `users` | H U | Principal/user snapshot (no password hashes). |
| `kill <pid>` | H U | Terminate a process by PID. |
| `prio <pid> <prio>` | U | Set process scheduling priority. |
| `affinity <pid> <core>` | U | Pin a process to a scheduler core. |
| `launch <path>` / `run <path>` | U | Launch a validated process image (format-guarded). |
| `process validate <path>` | H | Classify a flat/PIX/ELF64 image without launching. |
| `env` / `time <cmd>` / `echo <text>` | U | Shell environment, command timing, echo. |
| `reboot [confirm]` | U | PSCI `SYSTEM_RESET` (watchdog fallback). HTTP reboot is `/api/admin/reboot?confirm=1` or `:8081/?confirm=1`. |
| `update …` | U | OTA/update control (see also port-8082 protocol). |
| `clear` | U | Clear the console/echo state. |

---

## Filesystem, disk & storage

| Command | Surfaces | Description |
|---|---|---|
| `ls [path]` / `lsdir [path]` / `fsinspect [path]` | H U | Read-only WALFS directory listing / inspection. |
| `walfs status` / `fs status` | H U | Mount/root/super state, record count, WAL head, tree root. |
| **`walfs verify`** / **`disk verify`** / **`fs verify`** | **H** U | Verify WAL metadata + record-chain integrity: `super`, `wal_head`, `valid_records`, `crc_errors`, `header_errors`, `open_tx`, `scan_end`. *(HTTP wiring added so the health fields are visible remotely.)* |
| **`walfs compact`** / **`disk compact`** | **H** U | Non-destructive WAL compaction: rewrites only live records, resets WAL bloat. Reports new head/record count. |
| `walfs format confirm` | H U | **Destructive** reserved-base reformat (wipes WALFS; keystore root-of-trust is a separate partition and is untouched). |
| `disk [info\|sync\|read\|writezero]` | U | Low-level disk inspection/maintenance (detail via HDMI). |
| **`cachestats`** | **H U** | WAL inode/path, DNS, and ARP LRU cache hit/miss/evict telemetry. *(Newly wired: surfaces the previously unexposed `lru_stats`.)* |
| `db <wal:pack/card[/tail]> …` | H U | Picowal/Storage DB record access (canonical `kind:pack/card`). |
| `addr <kind:pack/card[/tail]>` | H U | Parse/canonicalize a PIOS resource address. Kinds: `wal tcp udp stream dev file`. |
| `cat / stat / find / mkdir / touch / rm / cp / mv / cpdir / df / pwd / cd` | U | Shell-style file operations and navigation. |
| `mount / umount / stream` | U | Mount points and stream resources. |
| `edit <path>` / `edit.pix <path>` | U | In-console editor (text / PIX). |
| `hexdump <args>` / `hexsec <lba>` | U | Hex dump of memory / a raw disk sector. |

---

## Networking & diagnostics

| Command | Surfaces | Description |
|---|---|---|
| `netstat` | H U | Live TCP listeners/sessions, owners, buffers, retries, firewall drops. |
| `netcfg` / `netcfg routes` / `netcfg neighbors` / `netcfg trace` | H U | Network summary, route table, ARP/neighbor table, last outbound route/MAC/UDP decision. |
| `dns resolve <host>` / `dns status` / `dns flush` | H U | Async A-record resolver (non-blocking). |
| `http get <ip\|host> [path] [port] [ms]` | H U | Console HTTP client (hostnames use the DNS cache). |
| `https get <ip\|host> [path] [port] [ms]` | H U | Console HTTPS client scaffolding. |
| `firewall` / `firewall list` | H U | List firewall rules (mutations are UART/TCP-only). |
| **`arp probe`** | **H U** | Send a gratuitous ARP (TX-path test); reports `requests_sent`, `learned`, `conflicts`. *(Newly wired: exposes `arp_probe`.)* |
| **`nic dump <on\|off>`** | **H U** | Toggle raw packet dump (pre-dispatch). *(Newly wired: exposes `nic_set_packet_dump`.)* |
| **`nic counters`** | **H U** | NIC `processed` / `dropped` / `firewalled` / `rate_limited` counters. |
| `wifi` | U | WiFi status (driver currently disabled on this board). |

---

## Security: crypto, TLS, X.509, ACME, keystore

| Command | Surfaces | Description |
|---|---|---|
| **`crypto selftest`** | **H U** | AES-GCM + nibble-table GHASH crypto selftest. *(Newly wired: exposes `crypto_selftest`.)* |
| `tls status` / `tls selftest` / `tls bridge` | H U | Kernel TLS diagnostics, record-layer selftest, PicoWeb HTTP bridge parse. |
| `x509 status` | H U | X.509 cert/key/binding state. |
| `x509 generate [cn]` / `x509 csr [cn]` / `x509 p256 [cn]` | H U | Generate self-signed dev cert / PKCS#10 CSR (Ed25519 or P-256). |
| `x509 bind` / `x509 import-self` / `x509 selftest` | H U | Bind cert to TLS, import self cert, run selftest. |
| `acme status` / `acme prepare <domain>` / `acme csrhex` | H U | ACME account/order state, CSR DER hex export. |
| `acme challenge <token> <keyauth>` / `acme clear` / `acme selftest` | H U | Manage HTTP-01 challenge state (served at `/.well-known/acme-challenge/<token>`). |
| `keystore status` / `keystore derive <label>` | H U | Sealed root-of-trust status / non-secret label fingerprint. |
| `sts status` | H U | PicoSTS token service status: secret present, user count, UTC set, live RNG/entropy status string. |
| `sts login <user> <pass> <aud> <tenant> [scope+scope]` | H U | Exercise `sts_login` (real code path). Stores the issued token in a diagnostics slot; prints granted scope + token length only (never the token bytes). |
| `sts validate <aud>` | H U | Validate the last stored token for an audience; prints tenant + scopes or the fail-closed error. |
| `sts tamper <aud>` | H U | Flip a signature byte of the stored token and validate — must be rejected (verifies tamper fail-closed). |
| `sts authz <scope> <aud>` | H U | Authorization-boundary probe: allow only if the stored token validates and carries `<scope>`. |
| `sts users [offset] [limit]` | H U | Drive the real `/api/sts/users` handler via a synthesized TLS request bearing the last stored token: exercises the `sts.admin` bearer gate (401/403 fail-closed) and bounded pagination (`total`/`offset`/`limit`/`next`). Prints the raw JSON response. |
| `sts gensecret` | H U | Provision an HS256 secret from the CSPRNG seam. **Fails closed** with the RNG status when no trusted entropy source is present. |
| `sts testusers` / `sts testsecret` | H U | **QEMU-only** deterministic test provisioning (fixed users + secret). Compiled out on production/hardware builds so known credentials never ship. |
| `brotli selftest` | H U | Verify in-tree Brotli encoder + PicoWeb micro-Brotli decoder. |

---

## Services, IPC & ABI

| Command | Surfaces | Description |
|---|---|---|
| `ksvc status` | H U | Kernel service registry: core ownership, priority, call/error/duration. |
| `ksvc selftest` | H U | Mailbox round-trip + pause/resume/fault/restart transitions. |
| `ksvc pause\|resume\|fault\|restart <id>` | H U | Service lifecycle controls. |
| `ipc bench [iterations]` | H U | IPC micro-benchmarks including cross-core FIFO IRQ delivery (`fifo_irq_delta`). |
| `abi status` / `abi selftest` | H U | Kernel/user ABI transition stage, SVC trap, entry contract, KPI shims. |
| `svc <args>` | U | Direct SVC/service shim invocation (debug). |
| `capsule …` / `obs …` | U | Capsule/observer subsystems (detail via HDMI). |

---

## Hardware & low-level

| Command | Surfaces | Description |
|---|---|---|
| `mem` / `mem analyze` | H U | Kernel image, raw-slot, per-core RAM, process memory layout. |
| `membench <addr>` | H | RAM write-latency micro-benchmark. **Do not run against the FB scanout (`0x3F400000`).** |
| `peek <addr> [1\|2\|4\|8]` | H U | Read live MMIO/RAM. |
| `poke <addr> <value> [1\|2\|4\|8]` | H U | Write live MMIO/RAM (admin/debug). |
| `dumpmem <addr> [bytes]` | H U | Dump a memory range. |
| `dma status` / `dma selftest` | H U | DMA channel registers, selftest, CB address mode. |
| `irq status` / `irq probe` / `irq selftest` | H U | IRQ counters, read-only GIC probes, delivery selftest. |
| `macbdiag` / `rxdiag` | H U | MAC RX ownership topology (`contig`, `after_gap`, `first_after`), hole/BNA/liveness recoveries, NIC/net/IRQ pipeline counters. |
| `break [core]` / `freeze status` / `regs <core>` / `resume [core]` | H U | Freeze/inspect/resume secondary cores; core 0 remains live as the control plane. |
| `irq cntpns confirm` / `irq cntpns step <n>` | H | Opt-in watchdog-protected CNTPNS/PPI30 delivery test / stepped probe. |
| `irq trace [reset]` / `irq sdtrace [wipe]` | H | IRQ dispatch / SD trace rings. |
| `rp1 irq` / `rp1 pci` | H | RP1 southbridge IRQ/PCIe diagnostics. |
| `rp1 irq arm-host6[-1]` / `arm-eth` / `raise-eth` / `clear` / `source-diag` / `pend-gic` | H | RP1 ETH IRQ arm/raise/clear/diagnose (HOST6 delivery). |
| `pcie aer [clear]` | H | PCIe Advanced Error Reporting snapshot. |
| `fb info` | H | Framebuffer geometry/scanout info. |
| `qpu status` / `qpu selftest` | H U | V3D/QPU dispatch diagnostics. |
| `tensor status` / `tensor selftest` | H U | Tensor NEON-fallback kernels. |
| `usb status\|reinit\|poll` | U | USB/xHCI controller diagnostics. |

---

## Watchdog & A/B boot control

| Command | Surfaces | Description |
|---|---|---|
| **`watchdog`** / **`watchdog status`** | **H** U | Watchdog `armed`, mode (reboot/halt), `timeout_ticks`, `trips`, `last_core`, `hw_remaining_ticks`. *(HTTP wiring added for parity with UART.)* |
| `bootctrl status` | H U | Stage0 A/B boot-control state (active/pending/tries/good_mask/generation). |
| `bootctrl clear-pending` / `bootctrl reset-a confirm` | H U | Repair boot-control without host raw-disk access. |
| `bootctrl test-invalid-b confirm` | H U | Non-destructive invalid-header rollback test. |

---

## UART/TCP shell scripting

The UART/TCP console additionally provides scripting primitives (HTTP terminal
does **not** expose these):

| Command | Description |
|---|---|
| `for … / foreach … / if …` | Loop and conditional constructs. |
| `batch <file>` / `source <file.pbc>` | Run a batch script / PicoScript bytecode file. |

---

## Newly wired commands (this revision)

The following capabilities existed in the kernel but had no operator command;
they are now wired to **both** surfaces and added to `help`:

| Command | Backing function | Module |
|---|---|---|
| `crypto selftest` | `crypto_selftest()` | `src/crypto.c` |
| `arp probe` | `arp_probe()` + `arp_get_stats()` | `src/arp.c` |
| `nic dump <on\|off>` | `nic_set_packet_dump()` | `src/nic.c` |
| `nic counters` | `nic_packet_counters()` | `src/nic.c` |
| `cachestats` | `walfs_cache_stats()` / `dns_cache_stats()` / `arp_cache_stats()` → `lru_stats()` | `src/walfs.c`, `src/dns.c`, `src/arp.c` |
| `walfs verify` / `disk verify` (HTTP) | `walfs_verify()` | `src/walfs.c` |
| `walfs compact` / `disk compact` (HTTP) | `walfs_compact()` | `src/walfs.c` |
| `watchdog` / `watchdog status` (HTTP) | `watchdog_status()` + `watchdog_hw_remaining_ticks()` | `src/watchdog.c` |

> Intentionally **not** wired: `el2_stage2_status()` is redundant with
> `capsule status`; that path remains internal.
