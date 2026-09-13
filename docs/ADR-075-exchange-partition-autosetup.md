# ADR-075: Automatic read-only PIOSXFER attachment

**Date:** 2026-09-13 · **Decider:** Owner (explicit request) · **Status:** Accepted

## Decision

After SD and WALFS setup, PIOS automatically evaluates the optional third
primary partition. `exchange_service` accepts only an immutable MBR snapshot
and the authoritative card sector count. It requires
`storage_layout_validate_exchange()` to accept the pre-created p1/p2/p3
layout, binds only validated p3 through `exchange_volume_policy`, and then requires
`fat32_exchange_core` to accept p3's real FAT32 BPB and exact eleven-byte
`PIOSXFER   ` label.

The platform SD adapter acquires the MBR snapshot before attachment. All later
filesystem callbacks are range-fenced to the selected p3 span, so p1 boot and
p2 WALFS sectors are inaccessible through the exchange service. Boot mounting
does not create, format, repartition, or write p3. Hardware starts with a
read-only attachment and only reports `exchange status`. QEMU retains the
bounded `xfer` mutation commands for its existing acceptance harness.

Legacy valid p1/p2 cards report exchange unavailable and boot normally. An
invalid p3 layout is rejected by the strict exchange gate but remains
diagnostic-only to WALFS; it cannot quarantine or block the core system. Every
attachment attempt starts from a cleared service state, so failed remounts
cannot leave a prior successful exchange mount visible.

## Consequences

The generic logic has an injected block callback host test covering legacy
absence, type and label rejection, a successful one-time read-only mount, no
automatic write/format, and p3-only callback access. A source gate prevents
the boot attachment path from acquiring mutation operations.

This does not authorize concurrent host/PIOS mounting, production writes,
crash consistency, automatic remediation, or partition-table changes. Those
require a separate owner decision.
