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
#define NVME_MAX_QUEUE_DEPTH         1024U

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
