/*
 * nvme.h - bounded, pure NVMe controller/Identify contracts
 *
 * This is the protocol foundation for an NVMe controller discovered behind
 * PCIe1. It does not touch MMIO, enable Bus Master, or submit hardware queues.
 */
#pragma once

#include "types.h"

#define NVME_PCI_CLASS_STORAGE       0x01U
#define NVME_PCI_SUBCLASS_NVM        0x08U
#define NVME_PCI_PROGIF_NVME         0x02U

#define NVME_ADMIN_IDENTIFY          0x06U
#define NVME_IDENTIFY_CONTROLLER     0x01U
#define NVME_IDENTIFY_BYTES          4096U
#define NVME_IDENTIFY_NAMESPACE       0x00U
#define NVME_MAX_QUEUE_DEPTH         1024U
#define NVME_ADMIN_TIMEOUT_MAX_MS    60000ULL
#define NVME_LBA_FORMAT_COUNT         16U
#define NVME_LBA_DATA_SHIFT_MIN       9U
#define NVME_LBA_DATA_SHIFT_MAX       16U
#define NVME_PRP_PAGE_BYTES           4096ULL
#define NVME_IO_QUEUE_COUNT           2U
#define NVME_IO_QUEUE_DEPTH           4U
#define NVME_IO_SLOT_COUNT            (NVME_IO_QUEUE_COUNT * NVME_IO_QUEUE_DEPTH)
#define NVME_IO_TIMEOUT_MAX_MS        60000ULL
#define NVME_IO_CONTRACT_MAGIC        0x4E564D45494F3031ULL
#define NVME_IO_CONTRACT_OWNER_CORE    0U

_Static_assert(NVME_PRP_PAGE_BYTES == 4096ULL,
               "initial NVMe PRP page contract is 4 KiB");

struct nvme_admin_cmd {
    u32 cdw0;
    u32 nsid;
    u64 reserved;
    u64 metadata;
    u64 prp1;
    u64 prp2;
    u32 cdw10;
    u32 cdw11;
    u32 cdw12;
    u32 cdw13;
    u32 cdw14;
    u32 cdw15;
} PACKED ALIGNED(64);

_Static_assert(sizeof(struct nvme_admin_cmd) == 64,
               "NVMe admin command must be 64 bytes");

struct nvme_admin_cqe {
    u32 result;
    u32 reserved;
    u16 sq_head;
    u16 sq_id;
    u16 command_id;
    u16 status;
} PACKED ALIGNED(16);

_Static_assert(sizeof(struct nvme_admin_cqe) == 16,
               "NVMe completion queue entry must be 16 bytes");

struct nvme_admin_handle {
    u16 command_id;
    u16 reserved;
    u32 generation;
};

struct nvme_admin_queue {
    u64 deadline_ms;
    u32 generation;
    u32 depth;
    u16 next_command_id;
    u16 active_command_id;
    u8 active;
    u8 reserved[43];
} ALIGNED(64);

_Static_assert(sizeof(struct nvme_admin_queue) == 64,
               "NVMe admin queue state must own one cache line");

enum nvme_admin_completion {
    NVME_ADMIN_COMPLETION_INVALID = 0,
    NVME_ADMIN_COMPLETION_SUCCESS,
    NVME_ADMIN_COMPLETION_FAILURE,
};

struct nvme_identify_info {
    u16 vendor_id;
    u16 subsystem_vendor_id;
    u8 serial[21];
    u8 model[41];
    u8 firmware[9];
    u8 ieee_oui[3];
    u8 max_data_transfer_exp;
    u32 version;
    u32 namespace_count;
};

/*
 * Identify Namespace (CNS 00h) is a 4096-byte little-endian NVMe response.
 * This contract accepts only metadata-free namespaces: neither extended
 * metadata (FLBAS bit 4) nor metadata bytes in the selected LBAF are usable
 * without a later, separately reviewed metadata data path.
 */
struct nvme_namespace_info {
    u64 nsze;
    u64 ncap;
    u64 nuse;
    u32 block_bytes;
    u16 metadata_bytes;
    u8 format_index;
    u8 lba_data_shift;
    u8 nlba_formats;
    u8 flbas;
    u8 metadata_capabilities;
    u8 read_only;
    u8 thin_provisioned;
};

/*
 * Geometry is an immutable copy selected from an Identify Namespace response.
 * `capacity_bytes` is the logical NSZE range, while `ncap` retains the
 * potentially smaller thin-provisioned allocation. The caller must first
 * validate CAP.MPSMIN is zero; only then does `max_data_transfer_exp` use the
 * fixed 4 KiB page formula: max_data_transfer_bytes = 2^(12 + exponent).
 * MDTS zero means unbounded and is intentionally rejected by this fixed-size
 * contract.
 */
struct nvme_namespace_geometry {
    u64 namespace_id;
    u64 nsze;
    u64 ncap;
    u64 nuse;
    u64 capacity_bytes;
    u64 max_data_transfer_bytes;
    u32 block_bytes;
    u32 max_blocks;
    u8 format_index;
    u8 lba_data_shift;
    u8 max_data_transfer_exp;
    u8 read_only;
    u8 thin_provisioned;
    u8 _reserved[3U];
} ALIGNED(64);

_Static_assert(sizeof(struct nvme_namespace_geometry) == 64U,
               "NVMe namespace geometry must own one cache line");

struct nvme_prp {
    u64 prp1;
    u64 prp2;
    u64 bytes;
    u32 page_count;
    u32 _reserved;
};

enum nvme_io_direction {
    NVME_IO_DIRECTION_READ = 1U,
    NVME_IO_DIRECTION_WRITE = 2U,
};

enum nvme_io_state {
    NVME_IO_STATE_FREE = 0U,
    NVME_IO_STATE_SUBMITTED,
    NVME_IO_STATE_COMPLETE,
    NVME_IO_STATE_FAILED,
    NVME_IO_STATE_RETIRED,
};

enum nvme_io_contract_state {
    NVME_IO_CONTRACT_EMPTY = 0U,
    NVME_IO_CONTRACT_ACTIVE,
    NVME_IO_CONTRACT_QUARANTINED,
    NVME_IO_CONTRACT_REMOVED,
    NVME_IO_CONTRACT_RETIRED,
};

enum nvme_io_fault {
    NVME_IO_FAULT_NONE = 0U,
    NVME_IO_FAULT_COMPLETION,
    NVME_IO_FAULT_TIMEOUT,
    NVME_IO_FAULT_CANCELLED,
    NVME_IO_FAULT_AER,
    NVME_IO_FAULT_REMOVED,
    NVME_IO_FAULT_GENERATION,
};

enum nvme_io_completion {
    NVME_IO_COMPLETION_INVALID = 0,
    NVME_IO_COMPLETION_SUCCESS,
    NVME_IO_COMPLETION_FAILURE,
};

/*
 * The caller supplies only numeric DMA authority. `dma_address` is never
 * converted to a CPU pointer; its exact `dma_bytes` must equal one request.
 * Queue IDs are the fixed deterministic IDs 1 and 2.
 */
struct nvme_io_request {
    u64 request_id;
    u64 lba;
    u64 dma_address;
    u64 dma_bytes;
    u32 block_count;
    u16 queue_id;
    u8 direction;
    u8 _reserved;
};

struct nvme_io_handle {
    u64 contract_id;
    u64 contract_epoch;
    u64 namespace_generation;
    u64 request_id;
    u64 generation;
    u16 command_id;
    u16 queue_id;
    u32 slot;
};

struct nvme_namespace_lease {
    u64 contract_id;
    u64 contract_epoch;
    u64 namespace_id;
    u64 generation;
    u64 geometry_fingerprint;
};

struct nvme_io_cqe {
    u32 result;
    u32 reserved;
    u16 sq_head;
    u16 sq_id;
    u16 command_id;
    u16 status;
} PACKED ALIGNED(16);

_Static_assert(sizeof(struct nvme_io_cqe) == 16U,
               "NVMe I/O completion queue entry must be 16 bytes");

/* Mutable slot controls never share a line with request payloads. */
struct nvme_io_control {
    u64 generation;
    u64 deadline_ms;
    u64 request_id;
    u32 state;
    u32 fault;
    u16 command_id;
    u16 queue_id;
    u8 _reserved[28U];
} ALIGNED(64);

struct nvme_io_payload {
    u64 request_id;
    u64 lba;
    u64 bytes;
    u64 dma_address;
    u64 prp1;
    u64 prp2;
    u32 block_count;
    u16 command_id;
    u8 direction;
    u8 page_count;
} ALIGNED(64);

struct nvme_io_contract_control {
    u64 magic;
    u64 contract_id;
    u64 contract_epoch;
    u64 namespace_generation;
    u64 last_request_id;
    u32 state;
    u32 fault;
    u32 owner_core;
    u16 next_command_id[NVME_IO_QUEUE_COUNT];
    u16 reported_sq_head[NVME_IO_QUEUE_COUNT];
    u8 _reserved[4U];
} ALIGNED(64);

struct nvme_io_contract {
    struct nvme_io_contract_control control;
    struct nvme_namespace_geometry geometry;
    struct nvme_io_control slots[NVME_IO_SLOT_COUNT];
    struct nvme_io_payload payloads[NVME_IO_SLOT_COUNT];
} ALIGNED(64);

_Static_assert(sizeof(struct nvme_io_control) == 64U,
               "NVMe I/O control must own exactly one cache line");
_Static_assert(sizeof(struct nvme_io_payload) == 64U,
               "NVMe I/O payload must have cache-line stride");
_Static_assert(sizeof(struct nvme_io_contract_control) == 64U,
               "NVMe I/O contract control must own one cache line");
_Static_assert(__builtin_offsetof(struct nvme_io_contract, geometry) == 64U,
               "NVMe geometry must not share contract control");
_Static_assert(__builtin_offsetof(struct nvme_io_contract, slots) == 128U,
               "NVMe mutable I/O controls require dedicated cache lines");
_Static_assert(__builtin_offsetof(struct nvme_io_contract, payloads) ==
               128U + NVME_IO_SLOT_COUNT * 64U,
               "NVMe immutable payloads must not share control lines");

static inline bool nvme_is_controller(u8 base_class, u8 subclass,
                                       u8 programming_interface)
{
    return base_class == NVME_PCI_CLASS_STORAGE &&
           subclass == NVME_PCI_SUBCLASS_NVM &&
           (programming_interface == 0U ||
            programming_interface == NVME_PCI_PROGIF_NVME);
}

static inline bool nvme_queue_depth_valid(u32 depth)
{
    return depth >= 2U && depth <= NVME_MAX_QUEUE_DEPTH &&
           (depth & (depth - 1U)) == 0U;
}

static inline u8 nvme_status_code(u16 status)
{
    return (u8)((status >> 1) & 0xFFU);
}

static inline u8 nvme_status_type(u16 status)
{
    return (u8)((status >> 9) & 0x07U);
}

static inline bool nvme_status_success(u16 status)
{
    return (status & 0xFFFEU) == 0U;
}

bool nvme_identify_buffer_valid(u64 address, u32 length);
bool nvme_build_identify_controller(struct nvme_admin_cmd *cmd,
                                    u64 data_address, u32 data_length);
bool nvme_identify_parse(const u8 *data, u32 length,
                         struct nvme_identify_info *out);
/* Queue initialization is one-shot over fresh all-zero storage. */
bool nvme_admin_queue_init(struct nvme_admin_queue *queue, u32 depth);
bool nvme_admin_submit(struct nvme_admin_queue *queue, u64 now_ms,
                       u64 timeout_ms, struct nvme_admin_handle *out);
enum nvme_admin_completion nvme_admin_complete(
    struct nvme_admin_queue *queue, const struct nvme_admin_handle *handle,
    const struct nvme_admin_cqe *cqe, u32 *result_out, u16 *status_out);
bool nvme_admin_expire(struct nvme_admin_queue *queue,
                       const struct nvme_admin_handle *handle, u64 now_ms);

bool nvme_identify_namespace_parse(const u8 *data, u32 length,
                                   struct nvme_namespace_info *out);
bool nvme_namespace_geometry_select(const struct nvme_namespace_info *info,
                                    u64 namespace_id,
                                    u8 max_data_transfer_exp,
                                    u8 controller_mpsmin,
                                    struct nvme_namespace_geometry *out);
bool nvme_namespace_range_valid(const struct nvme_namespace_geometry *geometry,
                                u64 lba, u32 block_count, u64 *bytes_out);
bool nvme_prp_build(u64 dma_address, u64 bytes, struct nvme_prp *out);

/*
 * One-shot fresh-storage initialization; it activates no controller hardware.
 * `contract_epoch` is a nonzero, externally assigned unique instance nonce.
 * Every lifecycle call is owned by and must execute on core 0.
 */
bool nvme_io_contract_init(struct nvme_io_contract *contract, u32 caller_core,
                           u64 contract_id, u64 contract_epoch);
bool nvme_io_namespace_lease_acquire(
    struct nvme_io_contract *contract, u32 caller_core,
    const struct nvme_namespace_geometry *geometry,
    struct nvme_namespace_lease *lease_out);
bool nvme_io_submit(struct nvme_io_contract *contract, u32 caller_core,
                    const struct nvme_namespace_lease *lease,
                    const struct nvme_io_request *request, u64 now_ms,
                    u64 timeout_ms, struct nvme_io_handle *handle_out);
enum nvme_io_completion nvme_io_complete(
    struct nvme_io_contract *contract, u32 caller_core,
    const struct nvme_io_handle *handle,
    /* `completed_bytes` is an adapter-validated byte count, not a CQE field. */
    const struct nvme_io_cqe *cqe, u64 completed_bytes);
bool nvme_io_expire(struct nvme_io_contract *contract, u32 caller_core,
                    const struct nvme_io_handle *handle, u64 now_ms);
bool nvme_io_cancel(struct nvme_io_contract *contract, u32 caller_core,
                    const struct nvme_io_handle *handle);
bool nvme_io_aer_quarantine(struct nvme_io_contract *contract,
                            u32 caller_core);
bool nvme_io_remove(struct nvme_io_contract *contract, u32 caller_core);
