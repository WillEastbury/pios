# PIOS System Documentation

## Contents

| Document | Description |
|----------|-------------|
| [architecture.md](architecture.md) | Hardware platform, core assignment, memory map, boot sequence |
| [drivers.md](drivers.md) | Hardware driver reference — registers, init sequences, APIs |
| [networking.md](networking.md) | Network stack design, security model, packet flow |
| [ipc.md](ipc.md) | Inter-core FIFO messaging protocol and message types |
| [api.md](api.md) | Full kernel API surface: userland kernel program interface (KPI) + kernel subsystem APIs |
| [primitives.md](primitives.md) | First-class kernel primitives model and target contract |
| [mmu.md](mmu.md) | MMU configuration, page tables, cache management |
| [kernel_dma_irq.md](kernel_dma_irq.md) | DMA/IRQ model and high-performance cross-core IPC transfer (span rings, barrier scope, DMA fast-path, core-0 idle) |
| [console-pix.md](console-pix.md) | Design for BusyBox-like `console.pix` interactive shell |
| [tensor.md](tensor.md) | NEON/QPU tensor compute, SIMD operations |
| [serial.md](serial.md) | Serial console setup — adapter pinout, wiring, terminal settings |
| [deployment.md](deployment.md) | SD card preparation, hardware wiring, boot process |
| [../CONSOLE.md](../CONSOLE.md) | Operator console, Web Admin, log tailing, OTA hot-flash, remote reboot, and firewall commands |
| [../DISKLAYOUT.md](../DISKLAYOUT.md) | FAT boot partition, partition-2 reserved system area, second-stage slot, and WALFS layout |
