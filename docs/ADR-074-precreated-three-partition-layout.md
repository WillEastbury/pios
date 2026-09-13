# ADR-074: Pre-created three-primary storage layout

**Date:** 2026-09-13 · **Decider:** Owner (explicit request) · **Status:** Accepted

## Decision

PIOS consumes only a pre-created MBR primary layout: index 0 is FAT32 boot
(`0x0B`/`0x0C`), index 1 is raw PIOS/WALFS (`0xDA`), index 2 is FAT32
PIOSXFER (`0x0B`/`0x0C`), and index 3 is empty. The shared pure
`storage_layout` validator verifies the signature, 512-sector capacity bounds,
spans, types, ordering, and p4 emptiness before production WALFS uses p2.
It publishes immutable numeric role facts. MBR status is observation only;
p1 may be active or inactive and no status bit grants storage authority.

The exact `PIOSXFER` label is a FAT32 adapter check after p3 selection, never
an MBR claim. At normal boot, after SD/WALFS setup, `exchange_service` receives
an immutable MBR snapshot and mounts valid p3 on every platform. Its callbacks
are permanently range-fenced to validated p3. The mount is read-only on
hardware; QEMU alone enables its existing bounded acceptance mutation commands.
No mount creates, formats, repartitions, or writes p3.

PIOS never writes an MBR, creates/repartitions a partition, or implicitly
formats a partition. `walfs format confirm` remains the sole explicit action
that initializes WALFS data inside validated p2. Existing p1/p2 cards classify
as legacy compatibility and retain p2 read/mount support while reporting
exchange missing; PIOS never upgrades their partition table.
