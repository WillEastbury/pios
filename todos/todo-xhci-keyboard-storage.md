# xHCI scope for keyboard and USB storage

Audit and implement the USB/xHCI stack in `C:\source\pios` with a deliberately narrow product goal:

1. USB HID keyboard support for boot/setup/system console
2. USB mass-storage support for simple directly attached USB storage devices

## Scope

This is **not** a general-purpose USB stack task.

The goal is to make the existing Raspberry Pi 5 / RP1 USB path reliable for:

- one directly attached USB keyboard
- and/or one directly attached USB mass-storage device

Avoid broadening scope unless it is required for those use cases.

## Context

- xHCI host controller code appears to be in `src\xhci.c`
- USB enumeration/core logic appears to be in `src\usb.c`
- Keyboard support appears to be in `src\usb_kbd.c`
- Storage support appears to be in `src\usb_storage.c`
- Current implementation appears intentionally minimal but may be missing important recovery, endpoint, and topology handling

## What to do

1. Inspect the current xHCI, USB core, keyboard, and storage code paths.
2. Identify what is already sufficient for:
   - HID boot keyboards
   - Bulk-Only USB storage
3. Identify the exact gaps likely to break real keyboards or USB storage devices on Pi 5 / RP1.
4. Implement the minimum reliable changes required for this narrow goal.
5. Keep the design intentionally simple where possible.

## Important design constraints

- Optimize for **reliable keyboard and storage support**, not generic USB completeness.
- It is acceptable to explicitly support:
  - no hubs
  - no composite devices unless already trivial
  - no hotplug beyond simple polling if that keeps things reliable
- If the current architecture only safely supports one device at a time, document that clearly and either:
  - preserve that intentionally, or
  - implement the smallest safe step needed for keyboard + storage together

## Implementation expectations

- Review whether `usb.c` incorrectly assumes only one device can ever exist.
- Review whether keyboard polling is incorrectly using a bulk-style path for interrupt endpoints.
- Review whether endpoint ring allocation is too small or too brittle.
- Add storage error recovery where needed:
  - timeout recovery
  - stall / halt recovery
  - Bulk-Only reset path if appropriate
- Review the xHCI event handling path for assumptions that are too fragile.
- Review DWC3 / RP1-specific init to ensure it is sufficient for real keyboards and USB sticks.
- Preserve the project's bare-metal style and existing code conventions.

## Questions to resolve in code

1. Can the current code support both a keyboard and a storage device at the same time?
2. If not, what is the smallest correct architectural change to enable that?
3. If simultaneous support is too invasive, can the code be made robust for one active USB role at a time?
4. What RP1/DWC3 initialization is still missing for dependable real-device bring-up?

## Deliverables

- Modified code
- Brief summary of what changed
- Explicit list of files changed
- Clear note on what is intentionally unsupported after the change
- Any remaining risks or follow-up items

## Important

Do not just describe the changes - make them in the codebase.
