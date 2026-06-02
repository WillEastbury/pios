# PIOS Disk Layout

PIOS uses the FAT boot partition only for the stable stage0 loader. The mutable operating system image and persistent storage live in partition 2.

## Partition overview

```text
Partition 1: FAT boot partition
  kernel8.img                 stable stage0 loader
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

0x000200..0x1FFFFF   Second-stage kernel image zone
                     Kernel image payload loaded by stage0.
                     Contains the second-stage loader plus core kernel code:
                     HDMI/VideoCore, RP1, NIC, DMA, IO, SD card, UART,
                     hardware init, scheduler, memory manager, etc.

0x200000..0x2C7FFF   TCP/IP stack area, 800 KiB
                     MAC, ARP, DHCP, DNS, ICMP, UDP, TCP, streams clients.

0x2C8000..0x2FFFFF   Firewall rules + static IP config area, 200 KiB

0x300000..0x3FFFFF   Admin Console HTTP server service area

0x400000..0x4FFFFF   Kernel debugger / dump / inspector area

0x500000..0x5FFFFF   User records

0x600000..0x7FFFFF   Circular hot log buffer

0x800000..0x8FFFFF   Kernel state crashdump zone

0x900000..0x9FFFFF   Reserved for future system services

0xA00000..end        WALFS structure and storage of packs/cards
```

## Boot and update flow

1. Pi firmware loads `kernel8.img` from the FAT partition.
2. `kernel8.img` is the stable stage0 loader.
3. Stage0 reads block 0 in partition 2 and validates the `PIOS` header.
4. Stage0 reads the second-stage payload from `0x000200..0x1FFFFF`.
5. Stage0 copies the second-stage image to RAM and jumps to it.
6. OTA hot-flash writes a new second-stage payload while the running kernel remains hot in RAM.
7. OTA commits by writing the valid header last.
8. Reboot uses PSCI `SYSTEM_RESET`; stage0 then loads the new second-stage image.

If the new image is corrupt and will not boot, recover by cold-patching the SD card from a host.

## Constants

The layout is codified in `include/walfs.h`:

```c
PIOS_RESERVED_BYTES         0x00A00000
PIOS_STAGE2_OFFSET          0x00000200
PIOS_STAGE2_END_OFFSET      0x001FFFFF
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
