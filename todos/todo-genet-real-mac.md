# Pi 5 Ethernet controller and real MAC address

Audit and implement a fix in `C:\source\pios` for Raspberry Pi 5 Ethernet bring-up, with special attention to the possibility that the current code is using the wrong controller entirely.

## Problem

The current codebase appears to assume Raspberry Pi 5 Ethernet uses **BCM GENET**, but there is a strong indication that Pi 5 actually uses an **RP1 Cadence MACB/GEM controller** instead.

That means the problem may be much larger than a fake/sample MAC fallback:

- the current Ethernet driver may be targeting the wrong hardware block
- the current MAC/PHY/DMA/MMIO assumptions may all be invalid on Pi 5
- the fake MAC fallback is still wrong, but the NIC may be fundamentally non-functional because the wrong controller is being driven

If the current driver is for the wrong controller, it should not be patched cosmetically. The implementation should be corrected at the controller level.

## Goal

Determine the actual Pi 5 Ethernet controller in this project, correct the driver path if needed, and ensure the NIC uses only a **real, valid MAC address** obtained from a legitimate source.

## Context

- Current Ethernet implementation appears to be in `src\genet.c`
- Current headers/base addresses appear to assume **GENET**
- New evidence suggests Pi 5 may instead expose Ethernet through **RP1 Cadence MACB/GEM**, possibly at or around `0x1F00100000`
- The current code appears to:
  1. try UMAC GENET registers
  2. try mailbox
  3. fall back to a hardcoded sample/default MAC
- If the controller is not GENET on Pi 5, then those register reads are not authoritative and the current fallback logic is built on the wrong premise

## What to do

1. Inspect the current MAC initialization path in `src\genet.c`.
2. Determine whether Raspberry Pi 5 Ethernet in this codebase is actually using the correct controller.
3. Verify whether Pi 5 Ethernet should be implemented with **GENET** or **Cadence MACB/GEM on RP1**.
4. Identify every source currently used for the MAC address.
5. Remove or disable the hardcoded placeholder fallback.
6. Require a valid MAC address before NIC initialization is considered successful.
7. Ensure invalid values are rejected, including:
   - all zeroes
   - broadcast FF:FF:FF:FF:FF:FF
   - multicast MACs
   - obvious placeholder/test values
8. If the real MAC cannot be read, fail loudly and clearly in logs.
9. Review whether mailbox fallback is valid and documented enough to keep.
10. Prefer the correct Pi 5 source of truth for the board MAC.
11. Review the rest of PHY/MAC bring-up to ensure the NIC is not reported healthy when MAC acquisition failed.
12. If the controller is wrong, replace or rework the Ethernet driver rather than patching the old GENET path.

## Implementation expectations

- Introduce explicit MAC validation logic.
- Do not allow networking to proceed with a fake address.
- Update initialization return behavior if MAC acquisition fails.
- Ensure any dependent network initialization handles this failure cleanly.
- Preserve existing style and bare-metal conventions.
- If Pi 5 uses MACB/GEM rather than GENET, update the driver architecture accordingly.

## Questions to resolve

1. Does Raspberry Pi 5 in this project actually use GENET, or is the correct controller MACB/GEM on RP1?
2. If Pi 5 does not use GENET, what code should replace `src\genet.c` and related assumptions?
3. If a real MAC cannot be obtained, should NIC initialization hard-fail? (Expected answer: yes.)
4. Is the mailbox MAC query actually valid and reliable, or should it be removed?
5. What is the correct board-specific source of truth for the MAC address on Pi 5?

## Deliverables

- Modified code
- Brief summary of what changed
- Explicit list of files changed
- Exact behavior when no valid MAC is available
- Explicit conclusion on whether GENET is the wrong controller for Pi 5 in this codebase
- Any remaining risks or assumptions

## Important

Do not just describe the changes - make them in the codebase.
