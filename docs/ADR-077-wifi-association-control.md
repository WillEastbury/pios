# ADR-077 — Offline Wi-Fi association control contract

**Date:** 2026-09-13
**Decider:** Owner
**Status:** Accepted
**Issue:** [#76](https://github.com/WillEastbury/pios/issues/76)

## Decision

Issue #76 adds `wifi_assoc_contract`, an offline, fixed-capacity Core-0
control model.  It accepts only copied observations: SDPCM-credit availability,
verified numeric control-publication evidence, timestamped association events,
and passive EAPOL-Key structural summaries.  It never retains an SSID,
credential, packet pointer, payload, or transport authority.

The only transition chain is `IDLE -> PREPARE -> WAIT_CREDIT ->
SET_SSID_PUBLISHED -> WAIT_ASSOC -> AUTHORIZED`, with `FAILED` and
`QUARANTINED` as fail-closed terminal states.  Every active call supplies a
strictly monotonic liveness sequence.  Missing or stale liveness evidence
cannot advance state or produce watchdog evidence.  A bounded step count and a
whole-operation deadline end stalled attempts deterministically.

Events are copied into an eight-record cache-line-isolated history ring.
`WLC_E_PSK_SUP` values 4 through 11 are preserved exactly, including the
documented M1/M2/M3/M4/group milestones.  EAPOL parsing validates a complete
802.1X EAPOL-Key frame, its declared/key-data lengths, descriptor version, and
nonzero replay counter, then classifies only M1, M3, G1, or other.  It performs
no WPA cryptography, key installation, or frame transmission.

All mutation checks both claimed and actual Core 0 and masks local IRQs over
each bounded state operation.  Handles bind an instance epoch and attempt
generation; reset invalidates all old handles.  Hardware enable is permanently
false.  No CYW43, kernel live-join, SDIO, MMIO, GPIO, or network code is
changed or linked.

## Consequences

A future owner-approved integration must separately define credential custody,
control-payload lifetime, SDPCM publication, event ingress, EAPOL/WPA
cryptography, recovery, and hardware acceptance.  ADR-077 authorizes none of
those actions.  The native and static-gate tests cover delayed credits,
unverified control publication, deadlines, cancellation, stale/late events,
PSK state preservation, EAPOL bounds, ring wrap, actual-core rejection,
generation invalidation, IRQ serialization, and the permanently false
activation gate.
