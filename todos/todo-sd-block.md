# SD raw block partition support

Audit and implement support in `C:\source\pios` for using the Raspberry Pi 5 SD card as:

1. Partition 1 = FAT32 boot partition for Pi firmware/kernel files
2. Partition 2 = raw block storage for PIOS/WALFS

## Context

- This is a bare-metal Pi 5 project.
- The goal is **not** to use FAT32 for the PIOS data path.
- The goal **is** to keep the mandatory Pi boot partition, then treat partition 2 as a raw block device.
- Existing raw SD access appears to be in `src\sd.c`.
- Existing cache/filesystem layers appear to be in `src\bcache.c` and `src\walfs.c`.
- Current code likely assumes whole-card raw access starting at LBA 0. That must change.
- WALFS and any higher-level storage code should operate relative to partition 2, not the whole card.
- The firmware boot partition must remain untouched by WALFS.

## What to do

1. Inspect the code paths for SD, bcache, WALFS, and any boot/storage metadata assumptions.
2. Identify exactly where the code assumes raw whole-device LBA addressing.
3. Implement partition-aware raw block support so partition 2 is discovered and used as the storage backing device.
4. Prefer MBR support first if simpler, but note GPT considerations if relevant.
5. Introduce a clean abstraction:
   - raw physical SD block read/write
   - storage partition base/length
   - logical WALFS block read/write relative to partition 2
6. Make the change surgical and consistent with existing code style.
7. Update any directly relevant documentation/comments.
8. Validate the build using the project's existing build flow.

## Implementation expectations

- Add code to parse the partition table from the SD card.
- Find partition 2 start LBA and size.
- Prevent WALFS from ever using raw LBA 0 of the full card as its superblock.
- Ensure bcache/WALFS use partition-relative addressing.
- Preserve existing behavior as much as possible except for the new partition offset handling.
- Call out any assumptions, especially around MBR vs GPT and partition type checks.
- If there are existing hardcoded sector assumptions (for superblock, WAL, root records, etc.), update them correctly.

## Deliverables

- Modified code
- Brief summary of what changed
- Explicit list of files changed
- Any remaining risks or follow-up items

## Important

Do not just describe the changes - make them in the codebase.
