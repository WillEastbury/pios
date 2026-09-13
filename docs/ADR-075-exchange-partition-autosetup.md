# ADR-075: Automatic PIOSXFER attachment and raw-p3 autoformat

**Date:** 2026-09-13 · **Decider:** Owner (explicit request) · **Status:** Superseded by [ADR-076](ADR-076-discovery-only-storage-layout.md)

## Decision

This ADR originally authorized an all-zero raw `0xDA` p3 to be formatted during
boot. The owner corrected that authorization before it was retained as the
storage-layout policy. **It is no longer implemented or operative.** ADR-076
supersedes it: PIOS only discovers existing layouts and never formats p3.

## Consequences

The formatter remains only in the host-test build of the callback-backed FAT32
core; no runtime service exposes or calls it. Current validation is specified
by ADR-076.
