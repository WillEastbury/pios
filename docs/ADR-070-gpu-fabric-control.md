# ADR-070 — Offline GPU-fabric control-plane contract

**Date:** 2026-09-12
**Decider:** Owner
**Status:** Accepted

## Decision

Issue #97 adds a fixed-capacity, Core-0-owned simulation/control contract for
at most eight GPU-fabric nodes. It records caller-supplied numeric facts only:
immutable node ID and epoch, generation, supported kernel mask, verified VRAM,
resident capacity/usage, health, and load. A node is eligible only when the
separate actual-GPU-validation fact is true and its health is OK.

Placements name an exact model ID/generation and explicit shard requirements.
They are deterministic (lowest eligible node ID), reserve resident capacity
atomically, and issue generation-backed handles. Failed placement makes no
state change. Removed or failed nodes invalidate dependent placements and
activation descriptors; clearing and a new placement request are the only
reassignment path—there is no automatic migration.

Activation routes contain only bounded numeric shard IDs, node generations,
sequence, opaque span start/bytes, and target credit capacity. They contain no
payload or pointer. Insufficient credit retains the same continuation until an
explicit retry provides enough credit. Duplicate, out-of-order, stale, or
malformed activation publication fails closed and quarantines the affected
placement where identity is available.

All public mutation is Core-0-only, bounded, allocation-free, and masks local
IRQs during the transition. `gpu_fabric_hardware_enable_allowed()` is
permanently false in this ADR.

## Consequences

This is simulation/control-plane logic only. It does not execute GPU work,
touch GPU or PCIe hardware, configure interrupts, exchange network packets,
interpret model data, or integrate DeveloperCLI/runtime paths. Any execution
adapter requires a separately approved ADR, explicit ownership/publication
rules, and hardware proof.

The focused native suite supplies more than 300 assertions over one through eight nodes,
capacity, affinity, heterogeneous kernels, missing verification, credit
backpressure/retry, stale identity, node failure/removal, and fixed-table
exhaustion. Its static gate rejects runtime integration and verifies protected
live paths have no diff.
