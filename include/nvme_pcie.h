#pragma once
#include "types.h"

enum nvme_pcie_state {
    NVME_PCIE_OFF = 0U,
    NVME_PCIE_DISCOVERED,
    NVME_PCIE_MMIO_READY,
    NVME_PCIE_QUARANTINED,
};

struct nvme_pcie_status {
    u64 cap;
    u64 bar_size;
    u64 cpu_base;
    u32 state;
    u32 fault;
    u32 bdf;
    u32 identity;
    u32 version;
    u32 command;
    u32 queue_entries;
    u32 doorbell_stride;
    u32 timeout_ms;
    u32 mpsmin;
    u32 mpsmax;
    u32 nvm_command_set;
    u32 identify_ok;
    u32 namespace_count;
    u32 controller_vendor;
    u32 identify_stage;
    u32 controller_status;
    u32 completion_status;
    u32 completion_cid;
    u64 namespace_blocks;
    u64 namespace_bytes;
    u64 max_transfer_bytes;
    u32 block_bytes;
    u32 max_transfer_blocks;
    u32 namespace_identify_ok;
    u32 io_ready;
    u8 serial[21];
    u8 model[41];
    u8 firmware[9];
    u8 _pad[1];
} ALIGNED(64);

_Static_assert(sizeof(struct nvme_pcie_status) == 256U,
               "NVMe PCIe status must have cache-line stride");

bool nvme_pcie_probe(void);
bool nvme_pcie_identify(void);
void nvme_pcie_revoke(void);
void nvme_pcie_status_get(struct nvme_pcie_status *out);
