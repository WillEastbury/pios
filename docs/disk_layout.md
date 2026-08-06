# Disk Layout & On-Disk Structures

Authoritative reference for everything PIOS stores on the SD card: the partition
map, the A/B raw-kernel slots, the boot-control sector, the OTA protocol, and the
WALFS / keystore / Picowal / principal record formats. Every structure is cited
to source.

This supersedes the high-level [../DISKLAYOUT.md](../DISKLAYOUT.md) for the boot
control + A/B raw-slot details (that file predates the A/B model). Constants live
in [`include/walfs.h`](../include/walfs.h).

Related: [boot.md](boot.md) (how stage0 consumes these structures).

---

## 1. Partition map

```
SD card
├── MBR (LBA 0)                      0x55AA signature; p2 start @ mbr[0x1CE+8]
├── Partition 1  — FAT32 boot        kernel8.img (stage0, board-detecting/
│                                    multi-platform -- see §1.1), PIOSSTG2.PKG
│                                    (may carry a Pi5 AND a BCM2837-family
│                                    payload simultaneously), config.txt,
│                                    start4.elf/fixup4.dat (Pi5), start.elf/
│                                    fixup.dat (Pi3/Pi Zero 2W), *.dtb
└── Partition 2  — PIOS system       raw kernel slots + control + records + WALFS
    ├── 0x000000 .. 0x9FFFFF  (10 MiB)  reserved system area  (§2)
    └── 0xA00000 .. end                 WALFS packs/cards/storage  (§5)
```

Partition 2 is discovered at runtime from the MBR; all offsets below are
**relative to the partition-2 start LBA**.

- `walfs_partition_lba()` → partition-2 start LBA (the raw-slot/control/records
  base).
- `walfs_part2_lba()` → WALFS data base = `partition2_start + 10 MiB/512`
  (`src/walfs.c:175-176`).
- `PIOS_RESERVED_BYTES = 10 MiB`, `WALFS_BOOT_SLOT_LBAS = 10 MiB/512 = 20480`
  sectors (`include/walfs.h:40-42,100-103`).

> The legacy read-only fallback slot starts at LBA `2048` when the MBR is
> unreadable. FAT update and boot-control writes are disabled unless partition 2
> passes full bounds validation.

### 1.1 Multi-platform stage0 and the two-tier size model

`kernel8.img` (stage0) now genuinely detects which board it is running on --
reading `MIDR_EL1` (a CPU-identification system register, not an MMIO
peripheral, so safe before any board-specific address is known) to
distinguish Cortex-A76 (Pi5) from Cortex-A53 (BCM2837-family: Pi3 B/B+/A+,
Pi Zero 2 W) -- see `include/board_detect.h`, `src/board_detect.c`. The SAME
`kernel8.img` therefore boots correctly on any of these boards from one FAT
partition.

This means `PIOSSTG2.PKG` (the FAT-resident package) and the raw-slot payload
it eventually produces are now governed by **two distinct size caps**, not
one:

| Cap | Value | Applies to | Constant |
|---|---|---|---|
| Per-platform raw-slot payload | `0x37FE00` (3.5 MiB) | the ONE payload stage0 extracts and writes into raw slot A | `PIOS_STAGE2_ZONE_BYTES` |
| Whole FAT package file | `16 MiB` | the FAT-resident `PIOSSTG2.PKG`, which may bundle multiple platforms' payloads | `PIOS_FAT_PACKAGE_MAX_BYTES` |

Stage0's `stage0_apply_fat_update()` loads the **whole** FAT file into the
staging buffer (`BOOT_STAGING_ADDR`, capped at `PIOS_FAT_PACKAGE_MAX_BYTES`),
then `select_stage2_image()` finds the entry matching the runtime-detected
platform id and extracts **only that payload subset** (capped at
`PIOS_STAGE2_ZONE_BYTES`) into raw slot A -- never the whole file. This is
what makes it safe for `PIOSSTG2.PKG` to be bigger than any single raw slot:
`tools/build_stage2_package.py --pi <pi5.bin> --bcm2837 <bcm2837.img> --out
PIOSSTG2.PKG` builds exactly such a combined package, and
`tools/stage0_pkg_check.py --platform pi5|bcm2837` validates either entry
independently.

Platform ids (`include/stage2_manifest.h`): `PIOS_STAGE2_PLATFORM_PI5=1`,
`PIOS_STAGE2_PLATFORM_QEMU_VIRT=2`, `PIOS_STAGE2_PLATFORM_BCM2837_FAMILY=6`
(Pi3 and Pi Zero 2 W share one entry -- both boot the identical compiled
image; see `include/platform.h` `PIOS_PLATFORM_PI3`/`PIOS_PLATFORM_PIZERO2W`).

---

## 2. Reserved system area (first 10 MiB of partition 2)

The reserved area carries the verified raw slot-A cache and boot control,
followed by persistent system records:

| Offset | Size | Region | Constant |
|---|---|---|---|
| `0x000000` | 3.5 MiB | **Boot slot A** (verified stage-2 package cache) | `PIOS_BOOT_SLOT_A_OFFSET` |
| `0x380000` | 512 B | **Boot-control sector** (`PBC0`) | `PIOS_BOOTCTRL_OFFSET` |
| `0x400000` | legacy | Legacy slot-B base; full packages are unsafe here | `PIOS_BOOT_SLOT_B_OFFSET` |
| `0x500000` | 1 MiB | User records (keystore sealed root, §6) | `PIOS_USER_RECORDS_OFFSET` |
| `0x600000` | 2 MiB | Circular hot-log buffer | `PIOS_HOT_LOGS_OFFSET` |
| `0x800000` | 1 MiB | Kernel-state crashdump zone | `PIOS_CRASHDUMP_OFFSET` |
| `0x900000` | 1 MiB | Reserved for future services | `PIOS_FUTURE_RESERVED_OFFSET` |
| `0xA00000` | — | WALFS base (§5) | `PIOS_WALFS_OFFSET` |

(`include/walfs.h:64-98`)

- Slot A is capped at `PIOS_BOOT_SLOT_BYTES = 0x380000` (3.5 MiB)
  (`include/walfs.h:67`). Within a slot, the 512-byte header is at offset 0 and
  the stage-2 payload starts at `PIOS_STAGE2_OFFSET = 0x200`
  (`include/walfs.h:64-65`).

> **Layout hazard.** The legacy slot-B base at `0x400000` plus a 3.5 MiB slot
> overlaps user records (`0x500000`) and hot logs (`0x600000`). The FAT updater
> therefore writes only slot A. The older TCP/IP/firewall/admin/debug header
> offsets are vestigial while the expanded stage2 package occupies their range.

### 2.1 Slot header (`PIOS`)

The first 512 bytes of each slot is the reserved-area header written by
`pios_fill_reserved_header()` (`src/kernel.c:3754-3775`). Little-endian u32
fields (`include/walfs.h:42-68`):

| Offset | Field | Notes |
|---|---|---|
| `0x00` | magic | `PIOS_RESERVED_HEADER_MAGIC = 0x50494F53` (`'PIOS'`) |
| `0x04` | stage-2 payload length | bytes of kernel image following the header |
| `0x08` | layout version | `PIOS_RESERVED_LAYOUT_VERSION = 1` |
| `0x0C` | reserved bytes | `10 MiB` |
| `0x10` | stage-2 offset | `0x200` |
| `0x14` | stage-2 zone bytes | |
| `0x18`–`0x44` | sub-zone offsets/sizes | tcpip/firewall/admin/debug/user-records/hot-logs/crashdump/future/walfs |

Stage0 validates `magic`, `image_len`, `layout_ver`, `stage2_off`,
`stage2_bytes` before loading a slot (`src/bootstrap.c:288-327`). On commit, the
valid header is written **last** (§4) so a partially written slot is never
selected.

---

## 3. Boot-control sector (`PBC0`)

A single 512-byte sector at partition-2 offset `0x200000` (LBA =
`walfs_partition_lba() + 0x200000/512`) that stage0 and the kernel use to drive
A/B selection (`src/kernel.c:3799-3818`).

Little-endian u32 layout (`include/walfs.h:71-85`):

| Offset | Field | Meaning |
|---|---|---|
| `0x00` | magic | `PIOS_BOOTCTRL_MAGIC = 0x50424330` (`'PBC0'`) |
| `0x04` | version | `PIOS_BOOTCTRL_VERSION = 1` |
| `0x08` | active_slot | currently-good slot (A=0, B=1) |
| `0x0C` | pending_slot | candidate slot, or `0xFFFFFFFF` (NONE) |
| `0x10` | tries_left | boot attempts remaining for the pending slot |
| `0x14` | last_boot | slot stage0 last jumped into |
| `0x18` | good_mask | bitmask of slots proven healthy (`1<<slot`) |
| `0x1C` | generation | monotonically increasing version |
| `0x20` | checksum | rolling hash over bytes `[0x00..0x1F]` |

- Slot ids: `SLOT_A=0`, `SLOT_B=1`, `SLOT_NONE=0xFFFFFFFF`, default tries `1`
  (`include/walfs.h:73-76`).
- Checksum: seed `0xB007C0DE`, `sum = (sum<<5) ^ (sum>>27) ^ byte` over the first
  `0x20` bytes (`src/kernel.c:3782-3787`). A block is valid only if magic,
  version, and checksum all match (`src/kernel.c:3790-3797`).

### State transitions (`src/kernel.c`)

| Function | Effect | Cite |
|---|---|---|
| `pios_bootctrl_init_good_at()` | active=A, pending=NONE, tries=0, last=A, good=`{A}`, gen=1 | `3821-3836` |
| `pios_bootctrl_mark_pending()` | pending=slot, tries=1, clears pending's good bit, gen++ | `3847-3867` |
| `pios_bootctrl_clear_pending()` | pending=NONE, tries=0, gen++ | `3869-3879` |
| `pios_bootctrl_reset_a()` | hard reset to A | `3881-3893` |
| `pios_bootctrl_mark_success()` | sets `good_mask |= 1<<last_boot`, pending=NONE, tries=0, gen++ | `3916-3932` |

Operator commands: `bootctrl status | clear-pending | reset-a confirm |
test-invalid-b confirm` (`src/kernel.c:3981-4012`); status fields printed by
`http_append_bootctrl_status()` (`src/kernel.c:3942-3979`). See
[boot.md](boot.md#6-health-gated-ab-success-and-rollback) for the health gate.

---

## 4. OTA update protocol

OTA writes the **inactive** slot while the running kernel stays hot, then arms it
as a pending candidate. Host tool: [`tools/pios_ota_update.py`](../tools/pios_ota_update.py).

- TCP port `8082` (`ADMIN_UPDATE_TCP_PORT`, `src/kernel.c:132`;
  `tools/pios_ota_update.py:102`).
- Query protocol `action=begin|chunk|commit|status|cancel`, `confirm=1` required
  (`tools/pios_ota_update.py:76-78,119-176`).
- Chunk size default `4096`, must be a multiple of 512
  (`tools/pios_ota_update.py:105-116`).

Kernel handler `http_build_kernel_update_response()` (`src/kernel.c:4217-4406`):

1. **begin** — validate total size, pick the target (inactive) slot via bootctrl,
   and **invalidate the slot header first** (`src/kernel.c:4274-4291`).
2. **chunk** — write the payload range only, to
   `slot_offset + 0x200 + offset`; reject out-of-order bytes
   (`src/kernel.c:4292-4333,4060-4067`).
3. **commit** — write the valid `PIOS` header **last**, then
   `pios_bootctrl_mark_pending()` (`src/kernel.c:4334-4354`).
4. **cancel** — deactivate the in-progress write and log (`src/kernel.c:4355-4362`).

Header-last commit guarantees a torn upload can never be selected by stage0,
and the A/B health gate (§3) rolls back a candidate that boots but is unhealthy.

---

## 5. WALFS (Write-Ahead Log filesystem)

WALFS is an append-only, crash-safe log of records on raw SD blocks. Latest
record for an inode wins; deletes are soft (`include/walfs.h:8-10`,
`src/walfs.c:8-9`).

- Magic `WALFS_MAGIC = 0x57414C46` (`'WALF'`), version 1, block size 512
  (`include/walfs.h:16-18`).
- Mounted base = `partition2_start + 10 MiB` (`src/walfs.c:41-47,175-179`).
- On-disk: **block 0 = superblock**, **block 1+ = sequential WAL records**
  (`src/walfs.c:4-9`).
- Root inode id = `WALFS_ROOT_INODE = 1` (`include/walfs.h:123`).

### Superblock (`struct walfs_super`, 512 B — `include/walfs.h:141-152`)

```c
struct walfs_super {
    u32 magic;          // 'WALF'
    u32 version;        // 1
    u32 block_size;     // 512
    u32 total_blocks;
    u64 wal_head;       // next free byte offset in the WAL region
    u64 tree_root;      // byte offset of the root inode record
    u32 record_count;
    u32 crc32;
    u8  label[32];
    u8  _reserved[448];
} PACKED;
```

### Record header + payloads (`include/walfs.h:154-190`)

```c
struct wal_record {     // every record begins with this header
    u32 magic;          // WALFS_REC_MAGIC = 0x5245434F ('RECO')
    u32 type;           // RECORD_INODE/DATA/DIRENT/DELETE/TX_BEGIN/TX_COMMIT
    u32 length;         // total record length incl. header
    u32 crc32;
    u64 seq;
    u64 timestamp;
    /* payload follows */
};

struct walfs_inode  { u64 inode_id, parent_id; u32 mode, flags; u64 size,
                      created, modified; u8 name[128]; };
struct walfs_data   { u64 inode_id, offset; u32 length; /* data[] follows */ };
struct walfs_dirent { u64 parent_id, child_id; u8 name[128]; };
struct walfs_delete { u64 inode_id; };
```

Record types: `RECORD_INODE=1`, `RECORD_DATA=2`, `RECORD_DIRENT=3`,
`RECORD_DELETE=4`, `RECORD_TX_BEGIN=5`, `RECORD_TX_COMMIT=6`
(`include/walfs.h:110-115`). Inode flags: `WALFS_DIR=0x01`, `WALFS_FILE=0x02`,
`WALFS_DELETED=0x80` (`include/walfs.h:118-120`).

### Recovery & integrity

Mount scans from the start, CRC-checks each record, and rebuilds the in-memory
index (`src/walfs.c:556-634`); the root inode is (re)created during format/repair
(`src/walfs.c:638-692`). `walfs_verify()` (`struct walfs_health`) and
`walfs_status()` (`struct walfs_status_snapshot`) expose integrity and mount
state (`include/walfs.h:192-216,254-260`); operator command `disk verify`.

---

## 6. Keystore / root of trust (`src/keystore.c`)

A single sealed 512-byte record holds the wrapped root key. It lives in the
user-records region: LBA = `walfs_partition_lba() + PIOS_USER_RECORDS_OFFSET/512`
(offset `0x500000`) (`src/keystore.c:78-90`).

```c
#define KEYSTORE_MAGIC 0x5254534B  /* 'KSTR' */            // keystore.c:11
struct keystore_record {                                   // keystore.c:21-30
    u32 magic, version, generation, flags;
    u8  nonce[12];
    u8  root_cipher[32];          // AES-GCM ciphertext of the root key
    u8  tag[16];                  // GCM auth tag
    u8  reserved[512 - 4*4 - 12 - 32 - 16];
} PACKED;
```

- The wrapping key is derived by **HKDF-SHA256** from the VideoCore board serial
  (`TAG_GET_BOARD_SERIAL` mailbox) with a fixed 32-byte salt and the info string
  `"PIOS keystore wrap key v1"` (`src/keystore.c:35-38,59-107`). If the serial is
  unavailable it falls back to a timer-derived value.
- `root_cipher`/`tag` are produced by **AES-GCM** sealing of the root key under a
  12-byte nonce; only nonce + ciphertext + tag + metadata are persisted — no
  plaintext root key is ever stored (`src/keystore.c:131-158`).
- Commands: `keystore status`, `keystore derive <label>`.

---

## 7. Picowal KV database (`src/picowal_db.c`)

A card/record key-value store layered on WALFS paths (not a separate on-disk
binary format).

- Key = `[card:10 bits][record:22 bits]`, packed `card<<22 | record`; card range
  `0..1023`, record range `0..4194303` (`include/picowal_db.h:5-18`,
  `src/picowal_db.c:283-296`).
- Backed by WALFS files `/var/picowal/c<card>/r<record>.rec`; init ensures
  `/var/picowal` exists (`src/picowal_db.c:7-17,250-341`).
- Console addressing: `wal:card/record` (see [../CONSOLE.md](../CONSOLE.md)).

---

## 8. Principal / user records (`src/principal.c`)

```c
struct principal {                       // include/principal.h:21-26
    u32 id;
    u8  name[32];
    u8  secret_hash[4];                  // iterated CRC32C-derived digest
    u32 flags;
};
```

The principal table is persisted to the Picowal record `/var/picowal/c1/r1.rec`
(deck 1, record 1), created on demand and rewritten on flush
(`src/principal.c:16-40,88-95`). Password verification uses iterated CRC32C
hashing (`src/principal.c:67-76`).

---

## Quick reference — on-disk constants

| Name | Value | Source |
|---|---|---|
| Slot A / control / legacy Slot B offsets | `0x000000` / `0x380000` / `0x400000` | `walfs.h:68-70` |
| Boot slot cap | `0x380000` (3.5 MiB) | `walfs.h:67` |
| Stage-2 payload offset (in slot) | `0x200` | `walfs.h:64` |
| Reserved-area header magic | `0x50494F53` `'PIOS'` | `walfs.h:44` |
| Boot-control magic | `0x50424330` `'PBC0'` | `walfs.h:71` |
| Boot-control checksum seed | `0xB007C0DE` | `kernel.c:3784` |
| WALFS magic / base offset | `0x57414C46` `'WALF'` / `0xA00000` | `walfs.h:16,98` |
| WALFS record magic | `0x5245434F` `'RECO'` | `walfs.h:109` |
| Keystore magic / offset | `0x5254534B` `'KSTR'` / `0x500000` | `keystore.c:11`, `walfs.h:93` |
| OTA TCP port | `8082` | `kernel.c:132` |
| Per-platform raw-slot payload cap | `0x37FE00` (3.5 MiB) | `walfs.h: PIOS_STAGE2_ZONE_BYTES` |
| Whole FAT package cap (may bundle multiple platforms) | `16 MiB` | `walfs.h: PIOS_FAT_PACKAGE_MAX_BYTES` |
| BCM2837-family platform id (Pi3/Pi Zero 2W, shared) | `6` | `stage2_manifest.h: PIOS_STAGE2_PLATFORM_BCM2837_FAMILY` |
