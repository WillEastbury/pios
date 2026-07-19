# PIOS Disk Layout

PIOS keeps the stable stage0 loader and an optional recovery/update package on FAT. The verified
boot cache and persistent storage live in partition 2.

## Partition overview

```text
Partition 1: FAT boot partition
  kernel8.img                 stable stage0 loader
  PIOSSTG2.PKG                checksummed read-only stage2 update source
  config.txt / firmware       Pi firmware boot files

Partition 2: PIOS system + storage partition
  0x000000..0x9FFFFF          reserved system area, 10 MiB
  0xA00000..end               WALFS packs/cards/storage
```

## Partition 2 reserved system area

All offsets below are relative to the start of partition 2.

```text
0x000000..0x0001FF   Block 0
                     PIOS reserved-area header:
                     signature, magic, layout version, active kernel length,
                     build/version, flags, optional CRC.

0x000200..0x37FFFF   Second-stage package cache (slot A)
                     Kernel image payload loaded by stage0.
                     Contains the second-stage loader plus core kernel code:
                     HDMI/VideoCore, RP1, NIC, DMA, IO, SD card, UART,
                     hardware init, scheduler, memory manager, etc.

0x380000..0x3801FF   Boot-control sector (`PBC0`)

0x400000              Legacy raw slot-B base. Do not install a full package here:
                      the current 3.5 MiB slot geometry overlaps the live regions below.

0x500000..0x5FFFFF   User records

0x600000..0x7FFFFF   Circular hot log buffer

0x800000..0x8FFFFF   Kernel state crashdump zone

0x900000..0x9FFFFF   Reserved for future system services

0xA00000..end        WALFS structure and storage of packs/cards
```

The older TCP/IP/firewall/admin/debug offsets encoded in the reserved header are metadata for
planned split services, not independently writable extents while the expanded stage2 cache occupies
`0x000200..0x37FFFF`. FAT stage0 therefore writes only raw slot A. Existing OTA slot-B support must
not be used for packages that extend into user-record or hot-log ranges until the layout is migrated.

## Boot and update flow

1. Pi firmware loads `kernel8.img` from the FAT partition.
2. Stage0 validates partition 2 and looks for root 8.3 file `PIOSSTG2.PKG`.
3. A changed, checksummed FAT package is written and byte-verified in raw slot A; its valid header
   is committed last, then boot control activates A.
4. If the FAT package is unchanged, stage0 preserves existing boot-control decisions (including OTA).
5. Stage0 loads the selected valid raw package, selects the Pi5 manifest entry, copies it to RAM,
   and jumps to it.
6. OTA hot-flash writes a new second-stage payload while the running kernel remains hot in RAM.
7. OTA commits by writing the valid header last.
8. Reboot uses PSCI `SYSTEM_RESET`; stage0 then loads the new second-stage image.

If the new image is corrupt and will not boot, recover by cold-patching the SD card from a host.

## WALFS initialization behavior

WALFS mounts only from the post-reserved base at partition 2 offset `0xA00000`.

At init, PIOS:

1. Discovers partition 2 from the MBR.
2. Validates the block-0 PIOS reserved-area header when present.
3. Computes WALFS capacity as `partition2_size - 10 MiB`, not whole-card size.
4. Reads the WALFS superblock at `partition2_start + 0xA00000`.
5. Refuses to auto-format if it finds a legacy WALFS superblock in the reserved area, so older data is not silently overwritten.
6. Formats only when no valid WALFS exists at the new base and no legacy superblock is detected.

## Constants

The layout is codified in `include/walfs.h`:

```c
PIOS_RESERVED_BYTES         0x00A00000
PIOS_STAGE2_OFFSET          0x00000200
PIOS_STAGE2_END_OFFSET      0x0037FFFF
PIOS_TCPIP_STACK_OFFSET     0x00200000
PIOS_TCPIP_STACK_BYTES      0x000C8000
PIOS_FIREWALL_CFG_OFFSET    0x002C8000
PIOS_FIREWALL_CFG_BYTES     0x00038000
PIOS_ADMIN_HTTP_OFFSET      0x00300000
PIOS_KERNEL_DEBUG_OFFSET    0x00400000
PIOS_USER_RECORDS_OFFSET    0x00500000
PIOS_HOT_LOGS_OFFSET        0x00600000
PIOS_CRASHDUMP_OFFSET       0x00800000
PIOS_FUTURE_RESERVED_OFFSET 0x00900000
PIOS_WALFS_OFFSET           0x00A00000
```
