# ADR-071: Offline FAT32 exchange-volume mutation core

**Status:** accepted for the offline #193 implementation slice.
**Owner:** repository owner direction (dedicated `PIOSXFER` exchange partition).

## Decision

Provide `fat32_exchange_core` as a pure, caller-owned FAT32 mutation module.
It accepts only injected exchange-attachment facts: a nonzero volume identity,
epoch, partition-relative LBA range, exact 11-byte volume label, and exact
512-byte read/write callbacks.  It does not discover partitions, select an
SD device, mount a live filesystem, or grant storage authority.

The initial format is FAT32 root-directory mechanics only: uppercase ASCII 8.3
file names, root files, FAT-copy mirroring, create/read/rewrite/append/delete/
rename/flush.  It validates the BPB, FAT ranges and copies, and a declared
FSInfo sector.  The attachment and BPB spans must be representable by the
32-bit callback LBA, and every cluster operation proves its entire sector
span before I/O.  It has no LFN support: a mutation of a short entry
immediately preceded by active LFN metadata is rejected, preserving that
metadata rather than orphaning it.  Valid, known FSInfo allocation hints are
written as unknown before the first mutation; the core never publishes a
successful mutation if that bounded write fails.  It has no live mount.

All calls are core-0, non-IRQ operations.  File capabilities carry the
attachment identity, epoch, mount generation, and mutation generation; stale,
wrong-volume, and wrong-owner calls fail closed.  State, I/O buffers, and
callbacks are caller-owned; there are no globals or heap allocation.

## Failure model

Callbacks must report an exact sector transfer or failure.  Both FAT copies
are read and compared before a FAT entry is updated, then written
synchronously.  An I/O error or mirrored-FAT mismatch faults the mounted
instance; it cannot continue issuing mutations.  A failed second FAT-copy
write can leave on-media copies divergent and is discovered on remount.

This is **not crash safe**: it supplies no journal, atomic multi-sector
transaction, recovery marker, host/PIOS concurrent ownership protocol, or
durability/barrier claim beyond the injected callback's exact-write contract.

## Consequences and non-goals

The module is host-tested against a fake, Windows/Linux-compatible FAT32
geometry, including fragmented allocation and fault injection.  It remains
unwired: no existing `fat32`, SD, WALFS, bootstrap, kernel, or live path is
modified.  A future approved exchange-policy/live integration must provide
ownership exclusion and a separate crash-consistency decision.
