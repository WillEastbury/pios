# ADR-072 — Offline cross-engine media admission/orchestration

**Date:** 2026-09-13
**Decider:** Owner
**Status:** Accepted
**Issue:** #169

## Decision

Provide a fixed-capacity (four plans, eight jobs per plan), pure control-plane
contract for media-engine orchestration. It is owned by core 0: each mutation
requires both a caller-core value of zero and the actual `core_id()` to be zero,
and serializes a transition with local IRQ masking and DAIF restoration.

Jobs receive immutable copied descriptions separate from their one-cache-line
mutable controls. A description carries a generation-backed job and plan
identity, one optional producer dependency, exact clock IDs, IOMMU resource,
GIC SPI, and a numeric DMA-span descriptor (ID, generation, start, bytes, and
capacity). The contract stores no payload or raw pointer.

Admission requires a current `media_engine_contract` lease and an exact match
to that engine's known resource facts. All DMA spans stay unique and
non-overlapping for the plan replay lifetime. Execution resources remain
exclusive while a job is incomplete. Therefore HEVC and PiSP-BE cannot
coexist: both need the one active IOMMU2 lease. Passive-only HVS and incomplete
PiSP-FE can never become active through this API.

An input consumer is admitted only after its producer completes with a valid
canary. Engine failure/removal, failed completion, invalid canary, or deadline
expiry faults the plan and the failed job's transitive dependents. There is no
automatic migration or substitution; a future attempt requires a new,
explicit plan and active lease.

Snapshots and a bounded, numeric replay ring expose plan/job states, fault
reasons, generations, engine kind, sequence, and time without changing media
execution behavior.

## Activation boundary

`media_admission_hardware_enable_allowed()` always returns false. The contract
does not access MMIO; configure or enable clocks, resets, GIC, IRQs, DMA,
IOMMU, GPU/V3D, framebuffer/display, or media hardware; nor does it alter
`media_hw.c` or any live media path. A future execution adapter requires a
separate owner-approved ADR and hardware acceptance evidence.

## Verification

`tests/test_media_admission.c` has more than 250 focused checks for ownership,
generation-stale handles, exact resource validation, bounded capacity,
non-overlap, HVS/PiSP-FE rejection, lease loss, dependency/canary ordering,
timeout, fault containment, replay, snapshots, retirement, and hardware
disablement. `tests/test_issue_169_media_admission_gate.py` rejects live media
integration and protected-path changes.
