# ADR-076: Discovery-only storage layout

**Date:** 2026-09-13 · **Decider:** Owner (explicit correction) · **Status:** Accepted

## Decision

PIOS discovers existing primary MBR layouts only. A legacy layout has p1 FAT32
boot and p2 raw PIOS/WALFS. A three-partition layout additionally has p3 as an
existing mountable FAT32 exchange volume. P3 must have MBR type `0x0B` or
`0x0C`; raw `0xDA` p3 is not an exchange volume and is unavailable.

The p3 FAT32 volume label, including `PIOSXFER`, is diagnostic rather than
authorization. A valid FAT32 p3 attaches regardless of its label. The
exchange boot attachment reads p3 only. It has no formatter, no formatting
state, and no boot-time write callback. Corrupt FAT32 p3 fails unavailable
without writes. P1 and p2 remain inaccessible through the exchange callbacks.

PIOS never writes the MBR, creates or repartitions a partition, or
automatically formats p3. Explicit `walfs format confirm` remains restricted
to WALFS data in validated p2. QEMU acceptance creates a preformatted p3
fixture; its explicit test-only transfer commands remain range-fenced to p3.

## Consequences

The storage-layout and exchange-service host tests prove valid unlabeled FAT32
attachment, rejection of raw p3, and no writes for raw or corrupt p3. QEMU
acceptance proves preformatted p3 attachment, persistence, label-independent
mounting, and no p3/p1/p2 changes when p3 is corrupt. This supersedes ADR-075.
