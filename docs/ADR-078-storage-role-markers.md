# ADR-078: Explicit storage role markers

**Date:** 2026-09-16 · **Decider:** Owner · **Status:** Accepted
**Issue:** [#193](https://github.com/WillEastbury/pios/issues/193)

## Decision

The three primary partitions may be exposed only by explicit role evidence:

- p1 is the FAT32 boot role;
- p2 is the WALFS role;
- p3 is the FAT32 exchange role.

An MBR type byte or the fact that a partition is raw is not sufficient to
authorize a role. A future storage adapter must supply a validated marker
read from the partition. The offline contract recognizes a FAT32 BPB marker
for p1/p3 and a WALFS superblock marker for p2. A PIOS reserved-area marker
may also be used for p2 once the adapter proves its version and bounds.

Formatting is not authorized by this ADR. A later operation may format only
an explicitly selected raw p2 as WALFS or raw p3 as FAT32 exchange, after
confirming the partition identity and role. It must never repartition a disk,
format p1, or infer a role from position alone.

The first implementation phase is offline-only: it validates exactly three
non-overlapping facts, their fixed p1/p2/p3 roles, marker presence, marker
versions, and generation-bound handles. It performs no storage I/O, mount,
format, or write.

## Consequences

This supersedes the role-authority portion of ADR-067 and the discovery-only
mount boundary in ADR-076. Live mounting, formatting, and shared storage
ownership remain separate follow-up decisions with hardware acceptance.
