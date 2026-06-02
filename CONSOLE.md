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

Tabs include:

- **Overview**: `/api/status` JSON, including `version` and `build`.
- **System**: terminal `status` output.
- **Netstat**: TCP diagnostics.
- **Processes**: process snapshot.
- **Users**: user/principal snapshot.
- **Logs**: operator log tail and process logs.
- **WALFS**: root WALFS browser.
- **Firewall**: live firewall rule panel.
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
