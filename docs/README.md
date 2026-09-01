# PIOS System Documentation

Start here: **[platforms.md](platforms.md)** (which boards and QEMU) then
**[architecture_system.md](architecture_system.md)** (the kernel model).
`architecture.md` is a short index, not a second architecture.

## Contents

| Document | Description |
|----------|-------------|
| [platforms.md](platforms.md) | **Boards and QEMU**: Pi 5, Pi 3 B/B+, Zero 2 W, QEMU `virt`; stage0 vs stage2; hardware matrix |
| [architecture.md](architecture.md) | Entry point: kernel contracts vs platform capabilities |
| [architecture_system.md](architecture_system.md) | Authoritative kernel model: cores, preemption, `airq`/`adrv`, FIFOs, isolation |
| [architecture_decision_log.md](architecture_decision_log.md) | Every significant architectural decision |
| [gotchas.md](gotchas.md) | Failed attempts, reverted changes, and traps |
| [boot.md](boot.md) | Boot sequence: two-stage A/B chain, `start.S` bring-up, `kernel_main` init order, health-gated rollback |
| [disk_layout.md](disk_layout.md) | Disk layout & on-disk structures: partition map, A/B raw slots, boot-control, OTA protocol, WALFS/keystore/principal records |
| [drivers.md](drivers.md) | Hardware driver reference — registers, init sequences, APIs |
| [network.md](network.md) | Authoritative network layering, core-0 communication, NIC/WiFi bring-up, meters, counters, and operator runbook |
| [network_stack.md](network_stack.md) | NIC/IP/TCP/TLS implementation |
| [networking.md](networking.md) | Stub — superseded by `network.md` |
| [wifi.md](wifi.md) | Stub — superseded by `network.md` and `platforms.md` |
| [ipc.md](ipc.md) | Inter-core FIFO messaging protocol and message types |
| [net_capsule_fifo.md](net_capsule_fifo.md) | Design draft: moving admin/web/TCP servers out of the kernel into FIFO/SWIRQ-woken user capsules, plus idle-core early-wake scheduling |
| [api.md](api.md) | Full kernel API surface: userland kernel program interface (KPI) + kernel subsystem APIs |
| [primitives.md](primitives.md) | First-class kernel primitives model and target contract |
| [mmu.md](mmu.md) | MMU configuration, page tables, cache management |
| [kernel_dma_irq.md](kernel_dma_irq.md) | DMA/IRQ model and high-performance cross-core IPC transfer (span rings, barrier scope, DMA fast-path, core-0 idle) |
| [console-pix.md](console-pix.md) | Design for BusyBox-like `console.pix` interactive shell |
| [commands.md](commands.md) | Command-line reference — every operator command across the HTTP `/api/terminal` and UART/TCP console surfaces |
| [tensor.md](tensor.md) | NEON/QPU tensor compute, SIMD operations |
| [serial.md](serial.md) | Serial console setup — adapter pinout, wiring, terminal settings |
| [deployment.md](deployment.md) | SD card preparation, hardware wiring, boot process |
| [size.md](size.md) | Measured image size analysis — stage0/stage2 totals, per-subsystem and per-object breakdown, raw-slot headroom |
| [../CONSOLE.md](../CONSOLE.md) | Operator console, Web Admin, log tailing, OTA hot-flash, remote reboot, and firewall commands |
| [../DISKLAYOUT.md](../DISKLAYOUT.md) | FAT boot partition, partition-2 reserved system area, second-stage slot, and WALFS layout |
