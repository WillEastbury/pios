# ADR-075: Automatic PIOSXFER attachment and raw-p3 autoformat

**Date:** 2026-09-13 · **Decider:** Owner (explicit request) · **Status:** Accepted

## Decision

After SD and WALFS setup, PIOS automatically evaluates the optional third
primary partition. `exchange_service` accepts only an immutable MBR snapshot
and the authoritative card sector count. It requires
`storage_layout_validate_exchange()` to accept the pre-created p1/p2/p3
layout, binds only validated p3 through `exchange_volume_policy`, and then
requires `fat32_exchange_core` to accept p3's real mountable FAT32 BPB. A
volume label is descriptive: valid FAT32 p3 attaches whether its label is
`PIOSXFER`, absent, or different.

The platform SD adapter acquires the MBR snapshot before attachment. All later
filesystem callbacks are range-fenced to the selected p3 span, so p1 boot and
p2 WALFS sectors are inaccessible through the exchange service. Boot does not
create or repartition a partition. If p3 has explicit raw PIOS type `0xDA` and its entire sector-zero boot sector
is all zero, boot formats only that p3 range as FAT32. This is a positive blank
media test, not a recognition heuristic: any nonzero byte, including damaged
or partial BPB geometry, root-cluster fields, labels, signatures, or unrelated
residue, fails closed without writes. The generic callback-backed formatter
writes fixed 512-byte metadata (primary/backup BPB, FSInfo pair, both FAT
mirrors, and root cluster), verifies its BPBs, and then remounts.

FAT-looking invalid p3 content fails closed rather than being overwritten;
FAT32-typed p3 cannot be autoformatted. Hardware remains read-only after the
optional format; `exchange status` reports `formatted` and `format_reason`.
QEMU retains its bounded `xfer` mutation commands for acceptance.

Legacy valid p1/p2 cards report exchange unavailable and boot normally. An
invalid p3 layout is rejected by the strict exchange gate but remains
diagnostic-only to WALFS; it cannot quarantine or block the core system. Every
attachment attempt starts from a cleared service state, so failed remounts
cannot leave a prior successful exchange mount visible.

## Consequences

The generic logic has an injected block callback host test covering legacy
absence, unlabeled valid FAT32 attachment, raw-p3 geometry/range format, and
nonblank raw-p3 residue rejection without p1/p2 writes, including damaged
root-cluster and other geometry with labels and signatures absent. The QEMU
acceptance boot provisions all-zero raw p3, exercises autoformat and remount,
then proves unlabeled attachment and corrupt-p3 no-write behavior.

This does not authorize concurrent host/PIOS mounting, production file writes,
crash consistency, broader automatic remediation, or partition-table changes.
