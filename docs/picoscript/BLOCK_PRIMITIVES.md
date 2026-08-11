# Raw block primitives

## Why

`Storage.*` cards and PicoWAL engines must not each invent OS I/O. They sit on a
single **block device** abstraction:

| Target | Backend | Geometry |
|--------|---------|----------|
| **Windows** | `pv_block_ops_mmap` — `CreateFile` + `CreateFileMapping` + `MapViewOfFile` | file bytes; logical block 4096 |
| **POSIX / WSL** | `pv_block_ops_mmap` — `open` + `mmap`/`msync`/`munmap` | same |
| **PIOS** | `pv_block_ops_walfs` — kernel WALFS LBA ops (installed backend) | fixed block size (typically 512) |
| **RP2350** | `pv_block_ops_rp2350` — caller-installed flash/SD LBA callbacks | fixed block size; caller-owned scratch |

```text
  Storage.* / picowal / Stage-0 corpus packer
                    │
              pv_block_dev
           ┌────────┼────────┐
           ▼        ▼        ▼
         mmap     mmap     WALFS
        (Win)   (POSIX)   (PIOS / sim)
```

## API

```c
#include "block/pv_block.h"

pv_block_dev d;
pv_block_open(&d, pv_block_ops_mmap(), "volume.bin",
              PV_BLOCK_READ|PV_BLOCK_WRITE|PV_BLOCK_CREATE|PV_BLOCK_MAP,
              1<<20);
pv_block_write(&d, offset, buf, len);
pv_block_read(&d, offset, buf, len);
void *p;
pv_block_map(&d, offset, len, /*writable=*/0, &p);  /* zero-copy */
pv_block_sync(&d);
pv_block_close(&d);
```

LBA form (WALFS-native):

```c
pv_block_read_blocks(&d, lba, buf, nblocks);
pv_block_write_blocks(&d, lba, buf, nblocks);
```

PicoScript uses the same mounted device through `Block.*`:

```text
Block.Ready(Rok)
Block.BlockSize(Rbytes)
Block.SizeLow(Rlo)
Block.SizeHigh(Rhi)
Block.SetOffset(Rlo, Rhi)
Block.Read(Rlength, Rspan)
Block.Write(Rspan, Rstatus)
Block.SetLba(Rlo, Rhi)
Block.ReadBlocks(Rcount, Rspan)
Block.WriteBlocks(Rspan, Rstatus)
Block.Resize(Rlo, Rhi, Rstatus)
Block.Sync(Rstatus)
Block.Status(Rstatus)
```

Offsets/LBAs remain 64-bit without widening PicoScript values. Transfers are
bounded to 16 KiB per hook call; larger operations are issued in chunks.

## PIOS / WALFS install

The kernel (or a host simulator) installs the backend once:

```c
static pv_walfs_backend be = {
    .ctx = kernel_vol,
    .block_size = 512,
    .read_blocks = svc_walfs_read_blocks,
    .write_blocks = svc_walfs_write_blocks,
    .sync = svc_walfs_sync,
    .block_count = svc_walfs_nblocks,
    /* optional zero-copy leases: */
    .map_range = svc_walfs_map,
    .unmap_range = svc_walfs_unmap,
    .scratch = walfs_scratch,
    .scratch_size = sizeof(walfs_scratch),
};
pv_walfs_install(&be);

pv_block_open(&d, pv_block_ops_walfs(), "sd0", PV_BLOCK_READ|PV_BLOCK_WRITE, 0);
```

## RP2350 install

The Pico SDK integration supplies flash, SDIO, or SPI-SD callbacks. The shared
adapter itself has no SDK dependency and performs no allocation:

```c
static uint8_t scratch[512];
static const pv_rp2350_backend rp = {
    .ctx = &sd_card,
    .block_size = 512,
    .block_count = SD_BLOCKS,
    .read_blocks = sd_read_blocks,
    .write_blocks = sd_write_blocks,
    .sync = sd_sync,
    .scratch = scratch,
    .scratch_size = sizeof scratch,
};
pv_rp2350_install(&rp);
pv_block_open(&dev, pv_block_ops_rp2350(), "sd0",
              PV_BLOCK_READ | PV_BLOCK_WRITE, 0);
pv_block_vm_install();
pv_block_vm_bind(&vm, &dev);
```

Host tests without PIOS:

```c
pv_walfs_sim_open("walfs.img", 512, 1024, PV_BLOCK_READ|PV_BLOCK_WRITE|PV_BLOCK_CREATE);
pv_block_open(&d, pv_block_ops_walfs(), "vol", PV_BLOCK_READ|PV_BLOCK_WRITE, 0);
```

## Bridge to PicoWAL portable engine

```c
pv_block_io_bridge io;
pv_block_io_bridge_init(&dev, &io);
/* map io.* onto pw_db_io_t read_at/write_at/sync/resize/size */
```

## Files

| File | Role |
|------|------|
| `host/block/pv_block.h` | API |
| `host/block/pv_block.c` | wrappers + default + bridge |
| `host/block/pv_block_mmap.c` | Win + POSIX mmap |
| `host/block/pv_block_walfs.c` | WALFS adapter + file simulator |
| `host/block/pv_block_rp2350.c` | allocation-free RP2350 callback adapter |
| `host/block/pv_block_vm.c` | public `Block.*` native VM hook bridge |
| `host/block/test_block.c` | smoke test |

## Build smoke

```powershell
cl /I host\block host\block\pv_block.c host\block\pv_block_mmap.c host\block\pv_block_walfs.c `
   host\block\test_block.c /Fe:build\test_block.exe
.\build\test_block.exe
```

## Next wiring

1. Point `storage_file` / a new `storage_block` at `pv_block_dev` instead of `FILE*`.
2. PicoWAL portable `pw_db_io_t` ← `pv_block_io_bridge`.
3. PIOS kernel: real `svc_walfs_*` filling `pv_walfs_backend`.
