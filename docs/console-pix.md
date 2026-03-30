# `console.pix` Design

Goal: a tiny interactive shell process that feels like BusyBox/bash/cmd, built on PIOS KPI (`struct kernel_api`), with deterministic behavior and bounded memory.

## Current Special-Case Runtime Hook

- `F3` now enters a kernel-hosted REPL mode on Core 0.
- `F4` opens a scheduler GUI for batch/timed jobs (keyboard-driven).
- Input is accepted from USB keyboard and UART RX.
- Output is mirrored to both UART and HDMI framebuffer console.
- This provides an immediate operator shell while `console.pix` is being built out.
- Current kernel REPL commands:
  - `help`, `echo`, `clear`, `time`, `pwd`, `cd`
  - `ps`, `kill <pid>`, `launch <path> [2|3]`, `run <path>`
  - `lsdir [path]`, `mkdir <path>`, `touch <path>`
  - `copy|cp <src> <dst>`, `cpdir <src_dir> <dst_dir>`, `mv <src> <dst>`
  - `cat <path>`, `stat <path>`, `rm <path>`
  - `find <dir> <needle>`, `hexdump <path> [max_bytes]`, `df`
  - `mount`, `umount` (currently informational stubs)
  - `stream <tcp|udp> <ip> <port> from <file|text|tty> ... to <console|file>`
  - `db key|put|putf|get|getf|del|list` (Picowal-style card/record DB on Core0->Core1 WALFS path)
- `batch add|at|every|run|stop|status|list|clear` (queued, timed, recurring scheduling with principal selection)
  - Supports optional core affinity (`1|2|3|auto`) and priority (`lazy|low|normal|high|realtime`)
- `svc add|start|stop|restart|run|pause|target|list|clear` (service supervisor with dependency + restart policy controls)
  - `svc add <name> <path> [dep|-] [target] [core] [priority] [principal] [restart] [max_restarts] [backoff_ms]`
  - targets: `default|rescue|all`, restart: `never|onfail|always`
- `update status|stage <slot> [tries]|success` (A/B rollout state: stage candidate slot with bounded boot attempts, then confirm success)
- `echo ... > <path>` (simple file write redirection)
- `prio <pid> <lazy|low|normal|high|realtime>` (update process priority class)
- `affinity <pid> <1|2|3>` (migrates process to target core by relaunching same executable/principal/priority under capsule policy, then retires old pid)
  - Scripting: `if`, `for`, `foreach`, `source`
  - Script extension: `.pis` (e.g. `source boot` resolves `/.../boot.pis` if present)
  - Env vars: `env list|get|set|pset|unset|save|load` (`pset` persists to Picowal deck 0, record 1)
  - `hexsec <lba>` (sector hex viewer)
  - `fsinspect [path]` (filesystem inspector)
- `netcfg [addnbr <ip> <mac>]` (network status/config)
- `obs` (security/network/EL2 observability snapshot: integrity checks/failures, capsule kills, firewall denies, drop counters, stage-2 faults)
- `netcfg set <ip|mask|gw|dns> <a.b.c.d>` + `netcfg apply`
- `netcfg dhcp <on|off> [timeout_ms]`
- `disk [info|sync|compact|verify|read <lba>|writezero <lba>]` (disk manager)
  - `disk writezero` now requires `--force`
- `edit|edit.pix <path>` opens a basic TUI editor (arrows move, insert/overwrite, delete/backspace, Ctrl+C/X/V line clipboard, Ctrl+S save, Ctrl+Q exit)

## Execution Model

- Binary: `console.pix` launched as a normal user process (`spawn("/bin/console.pix")` or F2 manager launch list).
- Entry: `void entry(struct kernel_api *k)`.
- Single-threaded REPL loop:
  - print prompt
  - read line (`getc`/`try_getc`)
  - tokenize
  - dispatch builtin or external command

## UX / Input

- Line editing v1: insert text, backspace, enter.
- History ring buffer (fixed depth, e.g. 16 lines).
- Deterministic max line length (e.g. 256 bytes), hard reject overflow.
- Text output via `print` (serial) and optional `fb_print` mirror mode.

## Command Model

Two command classes:

1. Builtins (fast, no child process):
- `help`, `clear`, `echo`, `time`, `whoami`, `auth`
- `ps` (PID/core/cpu%/mem from process UI surface)
- `kill <pid>`
- `launch <path> [core2|core3]`
- `pwd`, `cd`, `lsdir`, `mkdir`, `touch`, `copy|cp`, `cpdir`, `mv`
- `cat`, `stat`, `rm`, `find`, `hexdump`, `df`, `mount`, `umount`
- `stream`, `batch`, `svc`, `if`, `for`, `foreach`, `source`, `env`, `echo ... > file`
- `db` (card/record database commands backed by `/var/picowal`)
  - UDP surface: port `7001`, request header `[op,u8 ver,u16 rsv,u16 card,u32 record,u16 len,payload...]`
- later aliases: `ls`, `cat`, `stat`, `rm`, `write`
- `net` (link/status summary), `dns <host>`

2. External commands:
- `run <path>` → if `path` resolves to `.pis`, execute script in-shell; otherwise `spawn(path)`.
- Later: PATH-style lookup in `/bin`.

## Parsing Rules

- ASCII only, whitespace tokenization.
- Quoted args (`"..."`) in v2.
- No pipes/redirection in v1 (explicitly unsupported with error text).

## Process Manager Integration (F2)

- `C` on F2 opens/foregrounds console interaction (currently the F3 kernel REPL; later `console.pix`).
- `K` remains immediate kill in manager.
- `L` launches from shortlist; add `"/bin/console.pix"` as candidate.
- Console command `ps` should match manager fields:
  - PID, affinity core, CPU%, memory KiB, state.

Current behavior in kernel UI:

- `C` on F2 opens the F3 console mode immediately.

## Security / Capability Posture

- Console runs under its principal; operations naturally gated by KPI capability checks.
- No direct kernel pointers or unchecked memory operations.
- Explicit errors only (`ERR: access denied`, `ERR: not found`, `ERR: invalid args`).

## Memory/Size Budget

- Target static footprint: ~8–16 KiB code/data.
- Fixed buffers only:
  - input line: 256 B
  - argv table: 16 args
  - scratch output: 512 B
  - history: 16 x 256 B

## Milestone Plan

1. **M1 REPL core**: prompt, line input, `help/echo/clear/time`. ✅
2. **M2 FS + process ops**: `ps/kill/launch/run`, sector hex viewer, fs inspector. ✅
3. **M3 net/admin helpers**: `netcfg`, `disk` manager baseline. ✅
4. **M4 polish**: history navigation, better parser, optional fb mirror.

## Open Items

- Decide default principal for `console.pix` (`user0` vs admin-only shell).
- Confirm F2 `C` behavior: spawn once + focus, or spawn per press.
- Decide whether `kill` in console can target cross-core processes (recommended: yes via kernel helper).
