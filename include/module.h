#pragma once
#include "types.h"

#define MODULE_IMAGE_MAGIC          0x504D4F44U /* PMOD */
#define MODULE_IMAGE_VERSION        1U
#define MODULE_ABI_VERSION          1U
#define MODULE_VALIDATE_TOKEN       0x4D4FU
#define MODULE_MAX_IDENTITIES       8U
#define MODULE_SLOT_BYTES           0x00040000U
#define MODULE_ARENA_BYTES          0x00080000U
#define MODULE_DRAIN_TARGET_MS      30000U

#define MODULE_ID_B50               0U
#define MODULE_ID_TENSOR            1U
#define MODULE_ID_MAC               2U
#define MODULE_ID_IP                3U
#define MODULE_ID_TCP               4U
#define MODULE_ID_CAPSULE           5U
#define MODULE_ID_FILESYSTEM        6U
#define MODULE_ID_PROOF             7U

#define MODULE_CAP_PCIE             (1U << 0)
#define MODULE_CAP_DMA              (1U << 1)
#define MODULE_CAP_IRQ              (1U << 2)
#define MODULE_CAP_FIFO             (1U << 3)
#define MODULE_CAP_NETWORK          (1U << 4)
#define MODULE_CAP_STORAGE          (1U << 5)
#define MODULE_CAP_PICOSCRIPT       (1U << 6)

#define MODULE_OP_VALIDATE          0U
#define MODULE_OP_ADOPT             1U
#define MODULE_OP_QUIESCE           2U
#define MODULE_OP_CLEANUP           3U
#define MODULE_OP_CALL              16U

#define MODULE_SLOT_FREE            0U
#define MODULE_SLOT_STAGED          1U
#define MODULE_SLOT_ACTIVE          2U
#define MODULE_SLOT_RETAINED        3U

struct module_image_header {
    u32 magic;
    u16 version;
    u16 header_bytes;
    u32 module_id;
    u32 abi_version;
    u32 arena_schema;
    u32 capabilities;
    u32 artifact_generation;
    u32 code_bytes;
    u32 entry_offset;
    u8 sha256[32];
} PACKED;

struct module_services {
    u32 abi_version;
    u32 bytes;
    void *(*memset_fn)(void *, int, usize);
    void *(*memcpy_fn)(void *, const void *, usize);
    u32 (*crc32c_fn)(const void *, u32);
};

typedef u64 (*module_entry_fn)(u32 op, u64 a0, u64 a1, void *arena,
                               const struct module_services *services);
typedef u64 (*module_fallback_fn)(u32 op, u64 a0, u64 a1);

struct module_status {
    bool supported;
    bool switching;
    u32 module_id;
    u32 active_slot;
    u32 retained_slot;
    u32 staged_slot;
    u32 artifact_generation;
    u32 arena_schema;
    u32 capabilities;
    u64 dispatch_epoch;
    u64 active_calls;
    u64 roll_count;
    u64 rollback_count;
    u64 reclaim_count;
};

void module_init(void);
void module_service(void);
bool module_stage_image(const u8 *image, u32 image_bytes);
bool module_activate(u32 module_id);
bool module_rollback(u32 module_id);
u64 module_dispatch_call(u32 module_id, u32 op, u64 a0, u64 a1,
                         module_fallback_fn fallback);
void module_status_get(u32 module_id, struct module_status *out);
const char *module_command(u32 operation);

#define MODULE_COMMAND_STATUS   0U
#define MODULE_COMMAND_PROOF    1U
#define MODULE_COMMAND_ROLLBACK 2U
