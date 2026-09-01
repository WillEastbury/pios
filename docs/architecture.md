# Architecture

PIOS is a bare-metal OS. No Linux, no libc, no POSIX. The **same kernel
contracts** run on Raspberry Pi 5, Pi 3 B/B+, Pi Zero 2 W, and QEMU `virt`.
Hardware is a capability set (`PIOS_PLATFORM`); it is not “BCM2712 + RP1 only”.

## Where to read

| Document | Covers |
|---|---|
| [platforms.md](platforms.md) | **Boards and QEMU**: stage0 vs stage2, MIDR, IRQs, NIC/SD/Wi-Fi, memory maps, build flags |
| [architecture_system.md](architecture_system.md) | **Kernel model**: cores, quanta, mandatory preemption, `airq`/`adrv`, FIFOs, EL levels, isolation |
| [architecture_decision_log.md](architecture_decision_log.md) | Every significant decision, including what was rejected |
| [gotchas.md](gotchas.md) | Failed attempts and traps — check before “obvious” fixes |
| [boot_storage.md](boot_storage.md) | Two-stage boot, A/B OTA, WALFS |
| [network_stack.md](network_stack.md) / [network.md](network.md) | NIC backends, IP/TCP/TLS, operator map |

## Kernel model (one paragraph)

Core 0 is an event-driven reactor and is never a general scheduling target.
Cores 1–3 run **preemptive** process schedulers (GIC or QA7 timer armed;
EL0 entered with `I` clear; `ctx_switch` from IRQ after EOI). Cross-core
work is lock-free SPSC FIFOs plus SGI/SEV. Hardware IRQs enqueue; budgeted
software levels execute. Same PA must never be visible under conflicting
memory attributes.

Do not treat cores 1–3 as cooperative. Preemption is a hard invariant
(ADR-012 / ADR-021).

## Platforms (one paragraph)

Stage0 (`kernel8.img`) is runtime multi-platform: MIDR distinguishes A76
(Pi 5) from A53 (BCM2837 family), then firmware revision splits Pi 3 vs
Zero 2 W. Stage2 is compile-time per board. QEMU is a first-class target
with its own memory map (RAM from `0x40000000`), virtio-net/blk, and two
boot paths (direct `-kernel` and a real stage0 chain). Details and the
hardware matrix are in [platforms.md](platforms.md).
