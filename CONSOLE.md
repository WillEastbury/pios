# PIOS Console, Admin, and Operations Guide

PIOS exposes the same operator command set through several surfaces:

- **UART serial console** on the Pi 5 GPIO header, `115200 8N1`, no flow control.
- **HDMI mirror console**, which echoes UART input and command output to the framebuffer.
- **TCP debug console** on port `2323` after `unlock pios`.
- **Web Admin Console** on `http://192.168.218.101/`.
- **HTTP operator endpoints** on ports `8080`, `8081`, and `8082`.

## UART and HDMI console

Use a serial terminal on the host:

```text
COM17 at 115200 8N1, no flow control
```

The prompt is:

```text
PIOS Serial Console is online
Type Help for assistance!
ready>
```

Input typed on UART is echoed to both UART and HDMI. Command output is also mirrored to both surfaces.

### VT/ANSI terminal contract

The UART console is intended to behave like a clean remote terminal. Driver TX/RX chatter should stay quiet; diagnostics go to remote logs unless they are true warnings/errors.

Client consoles should support the following minimal VT/ANSI features:

```text
ESC[0m              reset style
ESC[2J              clear screen
ESC[H               cursor home
ESC[{row};{col}H    cursor move, 1-based
ESC[1;32;40m        bright green on black
ESC[1;36;40m        bright cyan on black
ESC[1;33;40m        bright yellow on black
ESC]0;title BEL      xterm/OSC window title
BEL / 0x07          bell
UTF-8               box drawing characters
```

PIOS emits UTF-8 single-line box drawing by default:

```text
┌────────────────────────┐
│ PIOS SECOND STAGE      │
└────────────────────────┘
```

The firmware keeps an ASCII fallback compile-time switch (`UART_VT_UTF8_BOXES=0`) for minimal serial clients:

```text
+------------------------+
| PIOS SECOND STAGE      |
+------------------------+
```

Suggested client-console behavior:

1. Open COM17 at `115200 8N1`, no flow control.
2. Set the terminal character set to UTF-8. In PuTTY: `Window -> Translation -> Remote character set -> UTF-8`.
3. Interpret ANSI SGR colour and cursor-control sequences above.
4. Interpret OSC title sequence `ESC]0;PIOS Admin Console BEL` where supported.
5. Render Unicode box drawing if available; fall back to ASCII if not.
6. Treat `BEL` as an optional audible/visual alert unless it terminates an OSC sequence.
7. Preserve scrollback.
8. Do not inject local echo unless the UART device echo is disabled; PIOS echoes input.
9. Keep command prompt rendering exactly as sent by PIOS.

Common commands:

```text
help
help status
help firewall
help core
status
ps
netstat
processes
users
firewall list
watchdog status
tls status
brotli selftest
reboot confirm
peek 0x1000FFF000 4
dumpmem 0x80000 128
```

`help core`, `help fs`, `help net`, `help svc`, and `help dev` are documentation groups only. Do not prefix commands with the group name. For example, use `status` and `ps`, not `core status ps`.

### Memory inspector commands

The debug/admin console includes direct memory inspection commands:

```text
peek <addr> [1|2|4|8]
poke <addr> <value> [1|2|4|8]
dumpmem <addr> [bytes]
```

Examples:

```text
peek 0x1000FFF000 4
poke 0x1000FFF000 0x1 4
dumpmem 0x80000 256
```

These commands directly access live kernel/device memory and are intended for admin/debug use only.

## First-boot OOBE

When the first-boot marker is missing, PIOS runs a setup sequence before normal operation.

It currently asks for:

- Locale: `en-GB` or `en-US`
- Keyboard layout: `uk` or `us`
- Timezone offset: UK/GMT (`UTC+0`) or US/Eastern (`UTC-5`)

The values are persisted in Picowal card `0`, record `3`:

```text
locale=en-GB
keyboard=uk
timezone_offset_minutes=0
```

The initial implementation intentionally supports only UK/US English options. More locales, keyboard maps, and daylight-saving rules are future work.

## TCP console

The TCP debug console listens on port `2323`.

```text
telnet 192.168.218.101 2323
unlock pios
firewall list
```

Port `2323` is allowed through the default inbound firewall. The unlock step is intentionally required because commands run on the live kernel.

## Web Admin Console

Open:

```text
http://192.168.218.101/
```

Tabs render as structured cards/tables and include a manual refresh button plus an auto-refresh checkbox with a millisecond interval control on live data tabs.

HTTP admin sessions clean up normal peer-close states without treating `CLOSE_WAIT` as an operator error, so routine browser refreshes should not flood the hot log with repeated `http-error` entries.

Tabs include:

- **Overview**: `/api/status` JSON, including `version` and `build`.
- **System**: terminal `status` output as summary cards/table plus raw text.
- **Netstat**: TCP diagnostics as a connection/session table.
- **Processes**: process snapshot table with PPID, arena/span memory columns, and graph section.
- **Users**: user/principal table.
- **Logs**: operator log tail and process logs in side-by-side cards.
- **WALFS**: root WALFS browser/status table.
- **Firewall**: live firewall rules table plus mutation examples.
- **Terminal**: green-screen HTTP terminal.
- **Admin**: links to log stream, OTA update, and reboot endpoints.

## HDMI post-boot dashboard

After initialization, HDMI switches from verbose boot/diagnostic text to a clean status dashboard that updates once per second.

It shows:

- `PIOS>` banner and second-stage build label.
- Uptime, heartbeat, and IP address.
- Per-core activity estimate and RAM use.
- Packet totals and firewall/drop counters.
- Listening TCP ports and owner labels.
- Process summary and a few active process rows.
- Lower warning/error hot-log tail.

Detailed logs and diagnostics should use the remote log endpoints rather than noisy HDMI/UART output.

The dashboard keeps the static frame on screen and refreshes only dynamic content rows to avoid full-screen flicker.

## Logs

Operator log stream:

```text
http://192.168.218.101:8080/logs
http://192.168.218.101:8080/logs?tail=24
http://192.168.218.101/api/admin/log-stream?tail=24
```

`ps`, `/api/terminal?cmd=processes`, HDMI process views, and the Web Admin Processes tab include parent PID (`PPID`) and a simple process graph/tree section.

Process memory columns:

```text
ACAP   arena capacity KiB available after loaded code and guard space
AUSED  current arena KiB in use by bump allocations plus rented spans
AHI    high-water/max arena KiB observed
ABUMP  KiB used through the process bump/sbrk arena
ASPAN  KiB currently held by rented spans
SCNT   active rented span count
```

PIOS userland also exposes arena span primitives through the kernel API:

```c
void *span_rent(u32 bytes, u32 align, u32 type);
i32 span_release(void *ptr);
```

Rented spans grow downward from the top of the process data arena, while `sbrk` grows upward. Allocation fails before the two regions can overlap.

Process memory protection:

- Loaded program image pages are mapped read-only/executable in the process slot.
- The process arena and stack start at the next 4 KiB page and are mapped read/write plus execute-never.
- The process slot uses W^X permissions; writable data pages are not executable.
- The low kernel/ABI region remains mapped for the current direct-call kernel API and is outside the process-slot W^X claim.

Use `tail=N` to limit the returned ring entries.

## Remote reboot

Remote reboot uses PSCI `SYSTEM_RESET` on Pi 5, with a legacy watchdog fallback.

From the local/TCP console:

```text
reboot confirm
```

The confirmation word is required to avoid accidental resets.

From HTTP:

```text
http://192.168.218.101:8081/?confirm=1
http://192.168.218.101/api/admin/reboot?confirm=1
```

## OTA hot-flash

The raw second-stage slot is updated over port `8082` using begin/chunk/commit.

Host helper:

```powershell
Set-Location C:\source\pios
python tools\pios_ota_update.py real_kernel.img --host 192.168.218.101 --reboot
```

Manual status:

```text
http://192.168.218.101:8082/?confirm=1&action=status
```

The FAT `kernel8.img` should be the stable stage0 loader. The second-stage kernel lives in the raw partition-2 slot and is what OTA updates.

## DMA diagnostics

DMA memcpy is self-tested during boot and can be checked from the console or Web Admin terminal:

```text
dma status
dma selftest
```

On BCM2712, PIOS uses the `dma32` controller with physical low-RAM DMA addresses, not the old `0xC0000000` legacy alias. The working memcpy path uses the 32-byte control-block format with shifted CB address mode; startup keeps the NEON fallback disabled only after the DMA selftest passes.

## Kernel TLS diagnostics

PIOS keeps TLS termination in kernel space. The current kernel layer provides a PSK/HKDF/AES-GCM record wrapper plus a PicoWeb-style bridge boundary that parses decrypted plaintext HTTP requests before handing them to a kernel service or future process-hosted webserver. AES rounds use ARM crypto instructions and GHASH uses a precomputed nibble table in the AES-GCM context.

Console/Web Admin terminal:

```text
tls status
tls selftest
tls bridge
```

`tls selftest` verifies that client and server key agreement produce compatible AES-GCM record keys. `tls bridge` parses a fixed sample HTTP request through the same complete/need-more/error convention used by PicoWeb's TLS-to-HTTP bridge.

## Brotli codec diagnostics

PIOS includes a no-external-dependency Brotli codec library. The encoder emits conservative stored Brotli streams; the decoder supports stored streams plus the compressed subset emitted by PicoWeb's micro-Brotli encoder. Large copies use the kernel SIMD copy path.

Console/Web Admin terminal:

```text
brotli selftest
```

The selftest round-trips a stored stream and decodes a compressed PicoWeb micro-Brotli fixture.

## Pack/Card resource addresses

PIOS accepts WAL-style resource addresses in the form:

```text
kind:pack/card[/tail]
```

`kind` is one of `wal`, `tcp`, `udp`, `stream`, `dev`, or `file`. Bare `pack/card` means `wal:pack/card`.

Examples:

```text
addr wal:0/3
addr tcp:0/80
addr udp:0/7001
addr stream:1/42
addr dev:0/1/uart0
addr file:0/12/etc/init.pis
```

For Picowal-backed storage, `wal:pack/card` maps to the existing database tuple `card=pack, record=card`, so old and new DB syntax both work:

```text
db get 0 3
db get wal:0/3
db put wal:0/3 hello
db list wal:0/0
```

Network resource addresses reserve `pack` as a namespace and use `card` as the TCP/UDP port number. For example, `tcp:0/2323` names the debug console listener.

## PicoScript compiler / assembler

PicoScript source files (`.pis`) are newline-delimited console commands. The kernel `source` command can run `.pis` directly or compiled `.pbc` bytecode.

Host compiler:

```powershell
python tools\picoscript.py compile script.pis script.pbc
python tools\picoscript.py asm script.pasm script.pbc
python tools\picoscript.py disasm script.pbc
```

The `.pbc` format stores validated command records and executes them through the existing console command dispatcher, so compiled scripts preserve current command behavior.

## User keystore / root of trust

PIOS seeds a sealed user root key into partition-2 reserved User Records block zero (`PIOS_USER_RECORDS_OFFSET`, currently LBA `12288` on the standard layout). The plaintext root key is never stored on disk and is not kept globally in memory.

The wrapping key is derived at boot from the board serial via HKDF-SHA256. The sealed record uses AES-GCM and stores only nonce, ciphertext, tag, and metadata. If VideoCore board-serial lookup fails, the status reports `serial=fallback` and the keystore remains unavailable or fallback-derived depending on boot state.

Console/Web Admin terminal:

```text
keystore status
keystore derive <label>
```

`keystore status` prints non-secret metadata and a root fingerprint. `keystore derive <label>` prints a non-secret fingerprint for a label-derived key; it does not expose key material.

## Firewall command

Defaults:

- Inbound: deny
- Outbound: allow
- New rules insert at index `0`, so the most recent rule wins before default service allows.

Syntax:

```text
firewall list
firewall reset
firewall clear
firewall default in deny out allow
firewall allow|deny <in|out|both> <tcp|udp|icmp|ip|arp> [port N|toport N|fromport N] [src SPEC] [dst SPEC]
```

`SPEC` supports:

```text
any
192.168.218.9
192.168.218.0/24
192.168.218.0/255.255.255.0
192.168.1.10-192.168.1.50
```

Examples:

```text
firewall allow in tcp port 2323 src 192.168.218.9
firewall deny in tcp port 80 src 192.168.218.0/24
firewall allow out udp toport 53 dst 192.168.218.1
firewall deny both ip src 192.168.1.10-192.168.1.50
```

## Build/version display

Each build generates:

```text
SECOND STAGE LOADER vYYYYMMDD.HHMMSS
```

The value is shown on:

- boot screen,
- UART boot text,
- `/api/status`,
- `:8080/` status,
- Web Admin Overview.
