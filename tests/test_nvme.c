#include <stdio.h>
#include <string.h>
#include "types.h"
#include "nvme.h"

static int failures;

#ifdef PIOS_HOST_CORE_ID_FN
static u32 host_core_id;

u32 pios_host_core_id(void)
{
    return host_core_id;
}
#endif

static void expect_true(const char *name, bool value)
{
    if (!value) {
        printf("FAIL %s\n", name);
        failures++;
    }
}

static void store_le64(u8 *p, u64 value)
{
    for (u32 i = 0U; i < 8U; i++)
        p[i] = (u8)(value >> (8U * i));
}

static bool io_contract_ready(struct nvme_io_contract *contract,
                              struct nvme_namespace_lease *lease,
                              bool read_only)
{
    struct nvme_namespace_info info = {
        .nsze = 100ULL,
        .ncap = 90ULL,
        .nuse = 10ULL,
        .block_bytes = 512U,
        .format_index = 0U,
        .lba_data_shift = 9U,
        .read_only = read_only ? 1U : 0U,
        .thin_provisioned = 1U,
    };
    struct nvme_namespace_geometry geometry;

    memset(contract, 0, sizeof(*contract));
    return nvme_namespace_geometry_select(&info, 7ULL, 1U, 0U, &geometry) &&
           nvme_io_contract_init(contract, 0U, 0x191ULL, 0x19100001ULL) &&
           nvme_io_namespace_lease_acquire(contract, 0U, &geometry, lease);
}

static struct nvme_io_request io_request(u64 request_id, u64 lba,
                                         u16 queue_id,
                                         enum nvme_io_direction direction)
{
    return (struct nvme_io_request){
        .request_id = request_id,
        .lba = lba,
        .dma_address = 0x100000ULL + request_id * 0x1000ULL,
        .dma_bytes = 512ULL,
        .block_count = 1U,
        .queue_id = queue_id,
        .direction = direction,
    };
}

static struct nvme_io_cqe io_cqe(const struct nvme_io_contract *contract,
                                 const struct nvme_io_handle *handle)
{
    return (struct nvme_io_cqe){
        .sq_head = contract->control.reported_sq_head[
            (u32)handle->queue_id - 1U],
        .sq_id = handle->queue_id,
        .command_id = handle->command_id,
        .status = 1U,
    };
}

static void test_namespace_geometry_and_prp(void)
{
    u8 identify[NVME_IDENTIFY_BYTES] = {0};
    struct nvme_namespace_info info;
    struct nvme_namespace_geometry geometry;
    struct nvme_prp prp;
    u64 bytes = 0ULL;

    store_le64(identify + 0U, 100ULL);
    store_le64(identify + 8U, 90ULL);
    store_le64(identify + 16U, 10ULL);
    identify[24] = 0x01U;
    identify[25] = 1U;
    identify[26] = 0U;
    identify[27] = 0x03U;
    identify[99] = 0x01U;
    identify[128] = 0U; identify[129] = 0U; identify[130] = 9U;
    identify[132] = 0U; identify[133] = 0U; identify[134] = 12U;
    expect_true("namespace Identify parses",
                nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info));
    expect_true("truncated namespace Identify rejects",
                !nvme_identify_namespace_parse(identify,
                                               NVME_IDENTIFY_BYTES - 1U,
                                               &info));
    expect_true("namespace capacity fields parse",
                info.nsze == 100ULL && info.ncap == 90ULL &&
                info.nuse == 10ULL && info.block_bytes == 512U);
    expect_true("thin-provisioned NSFEAT parses", info.thin_provisioned == 1U);
    expect_true("namespace selected LBA format parses",
                info.format_index == 0U && info.nlba_formats == 2U);
    expect_true("namespace MC and NSATTR offsets parse",
                info.metadata_capabilities == 0x03U && info.read_only == 1U);
    expect_true("namespace geometry selects bounded MDTS",
                nvme_namespace_geometry_select(&info, 7ULL, 1U, 0U,
                                               &geometry));
    expect_true("geometry has exact byte and block limits",
                geometry.capacity_bytes == 100ULL * 512ULL &&
                geometry.max_data_transfer_bytes == 8192ULL &&
                geometry.max_blocks == 16U);
    expect_true("valid last-LBA read range",
                nvme_namespace_range_valid(&geometry, 99ULL, 1U, &bytes) &&
                bytes == 512ULL);
    expect_true("zero block count rejected",
                !nvme_namespace_range_valid(&geometry, 0ULL, 0U, NULL));
    expect_true("LBA range overflow rejected",
                !nvme_namespace_range_valid(&geometry, 99ULL, 2U, NULL));
    expect_true("invalid LBA rejected",
                !nvme_namespace_range_valid(&geometry, 100ULL, 1U, NULL));
    geometry.nsze = ~0ULL;
    geometry.ncap = ~0ULL;
    expect_true("LBA plus count wrap rejected",
                !nvme_namespace_range_valid(&geometry, ~0ULL - 1ULL, 3U,
                                            NULL));
    geometry.nsze = 100ULL;
    geometry.ncap = 90ULL;
    expect_true("MDTS unbounded value rejected",
                !nvme_namespace_geometry_select(&info, 7ULL, 0U, 0U,
                                                &geometry));
    expect_true("MDTS exponent overflow rejected",
                !nvme_namespace_geometry_select(&info, 7ULL, 52U, 0U,
                                                &geometry));
    expect_true("non-4KiB MPSMIN rejected",
                !nvme_namespace_geometry_select(&info, 7ULL, 1U, 1U,
                                                &geometry));
    info.nsze = ~0ULL;
    info.ncap = ~0ULL;
    expect_true("capacity byte multiplication overflow rejected",
                !nvme_namespace_geometry_select(&info, 7ULL, 1U, 0U,
                                                &geometry));
    info.nsze = 100ULL;
    info.ncap = 90ULL;

    expect_true("unaligned single-page PRP preserves offset",
                nvme_prp_build(0x1004ULL, 32ULL, &prp) &&
                prp.prp1 == 0x1004ULL && prp.prp2 == 0ULL &&
                prp.page_count == 1U);
    expect_true("non-DWORD-aligned PRP1 rejects",
                !nvme_prp_build(0x1003ULL, 32ULL, &prp));
    expect_true("two-page PRP has aligned PRP2",
                nvme_prp_build(0x1FF0ULL, 32ULL, &prp) &&
                prp.prp2 == 0x2000ULL && prp.page_count == 2U);
    expect_true("three-page PRP rejects without list",
                !nvme_prp_build(0x1FF0ULL, 8193ULL, &prp));
    expect_true("PRP numeric span overflow rejected",
                !nvme_prp_build(~0ULL - 3ULL, 8ULL, &prp));

    identify[26] = 0x10U;
    expect_true("extended metadata format rejected",
                !nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info));
    identify[26] = 1U;
    identify[132] = 8U;
    expect_true("per-LBA metadata rejected",
                !nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info));
    identify[132] = 0U;
    identify[134] = 8U;
    expect_true("undersized LBA exponent rejected",
                !nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info));
    identify[134] = 17U;
    expect_true("oversized LBA exponent rejected",
                !nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info));
    identify[134] = 12U;
    identify[24] = 0U;
    expect_true("unthin unequal capacity rejects",
                !nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info));
    store_le64(identify + 8U, 100ULL);
    store_le64(identify + 16U, 100ULL);
    expect_true("unthin equal capacity parses",
                nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info) &&
                info.thin_provisioned == 0U);
    identify[24] = 1U;
    store_le64(identify + 8U, 90ULL);
    store_le64(identify + 16U, 10ULL);
    store_le64(identify + 8U, 101ULL);
    expect_true("capacity exceeding NSZE rejected",
                !nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info));
    store_le64(identify + 8U, 90ULL);
    store_le64(identify + 16U, 91ULL);
    expect_true("in-use capacity exceeding NCAP rejected",
                !nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info));
    store_le64(identify + 16U, 10ULL);
    store_le64(identify + 0U, 0ULL);
    expect_true("zero namespace size rejected",
                !nvme_identify_namespace_parse(identify, sizeof(identify),
                                              &info));
}

static void test_io_contract(void)
{
    struct nvme_io_contract contract;
    struct nvme_namespace_lease lease;
    struct nvme_io_handle handle;
    struct nvme_io_handle second;
    struct nvme_io_cqe cqe;
    struct nvme_io_request request;
    struct nvme_namespace_geometry geometry;
    struct nvme_namespace_lease replacement_lease;

    memset(&contract, 0, sizeof(contract));
#ifdef PIOS_HOST_CORE_ID_FN
    host_core_id = 1U;
    expect_true("actual nonzero core init rejects without mutation",
                !nvme_io_contract_init(&contract, 0U, 0x191ULL,
                                       0x19100001ULL) &&
                contract.control.magic == 0ULL);
    host_core_id = 0U;
#endif
    expect_true("wrong-core I/O init rejects without mutation",
                !nvme_io_contract_init(&contract, 1U, 0x191ULL,
                                       0x19100001ULL) &&
                contract.control.magic == 0ULL);
    expect_true("core-zero I/O ownership setup",
                io_contract_ready(&contract, &lease, false) &&
                contract.control.owner_core == 0U);
    geometry = contract.geometry;
    memset(&contract, 0, sizeof(contract));
    expect_true("core-zero I/O reinitializes for lease ownership test",
                nvme_io_contract_init(&contract, 0U, 0x191ULL,
                                      0x19100001ULL));
    expect_true("wrong-core lease rejects without mutation",
                !nvme_io_namespace_lease_acquire(&contract, 1U, &geometry,
                                                 &lease) &&
                contract.control.state == NVME_IO_CONTRACT_EMPTY);
    expect_true("core-zero lease acquires after wrong-core rejection",
                nvme_io_namespace_lease_acquire(&contract, 0U, &geometry,
                                                &lease));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("wrong-core submit rejects without mutation",
                !nvme_io_submit(&contract, 1U, &lease, &request, 0ULL, 10ULL,
                                &handle) &&
                contract.control.last_request_id == 0ULL);
    expect_true("wrong-core lifecycle setup submits on core zero",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    cqe = io_cqe(&contract, &handle);
#ifdef PIOS_HOST_CORE_ID_FN
    host_core_id = 1U;
    expect_true("actual nonzero core lifecycle operations reject",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_INVALID &&
                contract.control.state == NVME_IO_CONTRACT_ACTIVE &&
                contract.slots[handle.slot].state == NVME_IO_STATE_SUBMITTED);
    host_core_id = 0U;
#endif
    expect_true("wrong-core lifecycle operations reject without mutation",
                nvme_io_complete(&contract, 1U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_INVALID &&
                !nvme_io_expire(&contract, 1U, &handle, 10ULL) &&
                !nvme_io_cancel(&contract, 1U, &handle) &&
                !nvme_io_aer_quarantine(&contract, 1U) &&
                !nvme_io_remove(&contract, 1U) &&
                contract.control.state == NVME_IO_CONTRACT_ACTIVE &&
                contract.slots[handle.slot].state == NVME_IO_STATE_SUBMITTED);
    expect_true("core-zero completes wrong-core test request",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_SUCCESS);

    expect_true("request validation setup",
                io_contract_ready(&contract, &lease, false));
    request = io_request(1ULL, 99ULL, 1U, NVME_IO_DIRECTION_READ);
    request.block_count = 2U;
    request.dma_bytes = 1024ULL;
    expect_true("out-of-range I/O request rejects",
                !nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    request.dma_bytes = 511ULL;
    expect_true("I/O DMA capacity mismatch rejects",
                !nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("I/O deadline overflow rejects",
                !nvme_io_submit(&contract, 0U, &lease, &request, ~0ULL - 1ULL,
                               2ULL, &handle));
    expect_true("I/O success test setup",
                io_contract_ready(&contract, &lease, false));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("I/O read submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 10ULL, 20ULL,
                               &handle));
    second = handle;
    second.request_id++;
    cqe = io_cqe(&contract, &handle);
    expect_true("forged I/O handle rejected",
                nvme_io_complete(&contract, 0U, &second, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_INVALID);
    expect_true("I/O completion accepts exact bytes",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_SUCCESS);
    expect_true("released I/O handle is stale",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_INVALID);
    expect_true("request identity cannot be reused",
                !nvme_io_submit(&contract, 0U, &lease, &request, 20ULL, 20ULL,
                                &handle));

    request = io_request(2ULL, 89ULL, 1U, NVME_IO_DIRECTION_WRITE);
    expect_true("I/O write submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 20ULL, 20ULL,
                               &second));
    cqe = io_cqe(&contract, &second);
    expect_true("partial I/O success quarantines",
                nvme_io_complete(&contract, 0U, &second, &cqe, 511ULL) ==
                NVME_IO_COMPLETION_FAILURE &&
                contract.control.state == NVME_IO_CONTRACT_QUARANTINED);
    expect_true("quarantined contract rejects submissions",
                !nvme_io_submit(&contract, 0U, &lease, &request, 30ULL, 20ULL,
                                &handle));

    expect_true("SQ-head test setup", io_contract_ready(&contract, &lease, false));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("first outstanding SQ command submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    request = io_request(2ULL, 1ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("second outstanding SQ command submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &second));
    cqe = io_cqe(&contract, &second);
    cqe.sq_head = 2U;
    expect_true("out-of-order completion reports controller progress",
                nvme_io_complete(&contract, 0U, &second, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_SUCCESS &&
                contract.control.reported_sq_head[0] == 2U);
    cqe = io_cqe(&contract, &handle);
    cqe.sq_head = 2U;
    expect_true("older completion accepts retained controller head",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_SUCCESS &&
                contract.control.reported_sq_head[0] == 2U);

    expect_true("invalid SQ-head test setup",
                io_contract_ready(&contract, &lease, false));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("SQ-head completion submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    cqe = io_cqe(&contract, &handle);
    cqe.sq_head = NVME_IO_QUEUE_DEPTH;
    expect_true("out-of-range SQ head quarantines",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_FAILURE);

    expect_true("status test setup", io_contract_ready(&contract, &lease, false));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("failed-status completion submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    cqe = io_cqe(&contract, &handle);
    cqe.status = 0x0002U;
    expect_true("failed status quarantines",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_FAILURE);

    expect_true("completion test setup", io_contract_ready(&contract, &lease, false));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("malformed completion submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    cqe = io_cqe(&contract, &handle);
    cqe.command_id++;
    expect_true("malformed CID quarantines",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_FAILURE &&
                contract.control.fault == NVME_IO_FAULT_COMPLETION);

    expect_true("timeout test setup", io_contract_ready(&contract, &lease, false));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("deadline I/O submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 10ULL, 10ULL,
                               &handle));
    expect_true("early deadline rejected",
                !nvme_io_expire(&contract, 0U, &handle, 19ULL));
    expect_true("deadline quarantines",
                nvme_io_expire(&contract, 0U, &handle, 20ULL) &&
                contract.control.fault == NVME_IO_FAULT_TIMEOUT);

    expect_true("cancel test setup", io_contract_ready(&contract, &lease, false));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("cancellable I/O submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    expect_true("cancel quarantines deterministically",
                nvme_io_cancel(&contract, 0U, &handle) &&
                contract.control.fault == NVME_IO_FAULT_CANCELLED);

    expect_true("AER test setup", io_contract_ready(&contract, &lease, false));
    expect_true("AER quarantines empty contract",
                nvme_io_aer_quarantine(&contract, 0U) &&
                contract.control.fault == NVME_IO_FAULT_AER);
    expect_true("removal after AER is deterministic",
                nvme_io_remove(&contract, 0U) &&
                contract.control.state == NVME_IO_CONTRACT_REMOVED &&
                contract.control.fault == NVME_IO_FAULT_REMOVED);

    expect_true("read-only setup", io_contract_ready(&contract, &lease, true));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_WRITE);
    expect_true("read-only namespace rejects writes",
                !nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                                &handle));
    request.direction = NVME_IO_DIRECTION_READ;
    request.block_count = 0U;
    expect_true("zero-length read rejects",
                !nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                                &handle));

    expect_true("reuse test setup", io_contract_ready(&contract, &lease, false));
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("first reusable slot submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    cqe = io_cqe(&contract, &handle);
    expect_true("first reusable slot completes",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_SUCCESS);
    contract.slots[handle.slot].generation = ~0ULL;
    request = io_request(2ULL, 1ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("exhaustible generation submits once",
                nvme_io_submit(&contract, 0U, &lease, &request, 20ULL, 10ULL,
                               &second));
    cqe = io_cqe(&contract, &second);
    expect_true("exhausted slot completes",
                nvme_io_complete(&contract, 0U, &second, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_SUCCESS &&
                contract.slots[second.slot].state == NVME_IO_STATE_RETIRED);

    expect_true("CID wrap test setup", io_contract_ready(&contract, &lease, false));
    contract.control.next_command_id[0] = 0xFFFFU;
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("CID at u16 maximum submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle) &&
                handle.command_id == 0xFFFFU);
    contract.control.next_command_id[0] = 0xFFFFU;
    request = io_request(2ULL, 1ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("wrapped CID skips outstanding CID",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &second) &&
                second.command_id != handle.command_id &&
                second.command_id == 1U);

    expect_true("instance epoch ABA setup",
                io_contract_ready(&contract, &lease, false));
    geometry = contract.geometry;
    request = io_request(1ULL, 0ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("pre-reinitialization I/O submits",
                nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &handle));
    cqe = io_cqe(&contract, &handle);
    memset(&contract, 0, sizeof(contract));
    expect_true("zero instance epoch rejects",
                !nvme_io_contract_init(&contract, 0U, 0x191ULL, 0ULL));
    expect_true("same contract ID accepts distinct instance epoch",
                nvme_io_contract_init(&contract, 0U, 0x191ULL, 0x19100002ULL) &&
                nvme_io_namespace_lease_acquire(&contract, 0U, &geometry,
                                                &replacement_lease));
    expect_true("old lease fails after zeroed reinitialization",
                !nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                               &second));
    expect_true("old handle fails after zeroed reinitialization",
                nvme_io_complete(&contract, 0U, &handle, &cqe, 512ULL) ==
                NVME_IO_COMPLETION_INVALID);
    expect_true("replacement lease carries instance epoch",
                replacement_lease.contract_epoch == 0x19100002ULL);

    expect_true("queue capacity setup", io_contract_ready(&contract, &lease, false));
    for (u64 id = 1ULL; id <= NVME_IO_QUEUE_DEPTH; id++) {
        request = io_request(id, id - 1ULL, 1U, NVME_IO_DIRECTION_READ);
        expect_true("fixed queue slot submits",
                    nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                                   &handle));
    }
    request = io_request(9ULL, 8ULL, 1U, NVME_IO_DIRECTION_READ);
    expect_true("fixed queue exhaustion rejects",
                !nvme_io_submit(&contract, 0U, &lease, &request, 0ULL, 10ULL,
                                &handle));
}

static void test_admin_queue(void)
{
    struct nvme_admin_queue queue;
    struct nvme_admin_handle first;
    struct nvme_admin_handle second;
    struct nvme_admin_cqe cqe = {0};
    u32 result = 0U;
    u16 status = 0U;

    memset(&queue, 0, sizeof(queue));
    expect_true("invalid queue depth rejected",
                !nvme_admin_queue_init(&queue, 3U));
    expect_true("admin queue initialises", nvme_admin_queue_init(&queue, 16U));
    expect_true("admin queue reinit rejected",
                !nvme_admin_queue_init(&queue, 16U));
    expect_true("timeout zero rejected",
                !nvme_admin_submit(&queue, 100U, 0U, &first));
    expect_true("oversized timeout rejected",
                !nvme_admin_submit(&queue, 100U,
                                   NVME_ADMIN_TIMEOUT_MAX_MS + 1ULL, &first));
    expect_true("deadline overflow rejected",
                !nvme_admin_submit(&queue, ~0ULL - 1ULL, 2U, &first));
    expect_true("first admin command submits",
                nvme_admin_submit(&queue, 100U, 50U, &first));
    expect_true("second active command rejected",
                !nvme_admin_submit(&queue, 101U, 50U, &second));
    expect_true("early expiry rejected",
                !nvme_admin_expire(&queue, &first, 149U));

    cqe.command_id = first.command_id;
    cqe.sq_id = 0U;
    cqe.sq_head = 1U;
    cqe.result = 0xAABBCCDDU;
    cqe.status = 1U;
    expect_true("successful completion accepted",
                nvme_admin_complete(&queue, &first, &cqe, &result, &status) ==
                NVME_ADMIN_COMPLETION_SUCCESS);
    expect_true("completion result returned", result == 0xAABBCCDDU);
    expect_true("completion status returned", status == 1U);
    expect_true("released handle is stale",
                !nvme_admin_expire(&queue, &first, 200U));

    expect_true("second admin command submits",
                nvme_admin_submit(&queue, 200U, 20U, &second));
    cqe.reserved = 1U;
    cqe.command_id = second.command_id;
    expect_true("reserved CQE field rejected",
                nvme_admin_complete(&queue, &second, &cqe, NULL, NULL) ==
                NVME_ADMIN_COMPLETION_INVALID);
    cqe.reserved = 0U;
    cqe.command_id = second.command_id;
    cqe.sq_head = 16U;
    expect_true("out-of-range CQ head rejected",
                nvme_admin_complete(&queue, &second, &cqe, NULL, NULL) ==
                NVME_ADMIN_COMPLETION_INVALID);
    cqe.sq_head = 2U;
    cqe.sq_id = 1U;
    expect_true("non-admin CQE rejected",
                nvme_admin_complete(&queue, &second, &cqe, NULL, NULL) ==
                NVME_ADMIN_COMPLETION_INVALID);
    cqe.sq_id = 0U;
    cqe.status = 0x0002U;
    expect_true("valid failed completion releases command",
                nvme_admin_complete(&queue, &second, &cqe, NULL, NULL) ==
                NVME_ADMIN_COMPLETION_FAILURE);

    expect_true("third admin command submits",
                nvme_admin_submit(&queue, 300U, 20U, &second));
    expect_true("deadline expiry releases command",
                nvme_admin_expire(&queue, &second, 320U));
    expect_true("expired handle is stale",
                !nvme_admin_expire(&queue, &second, 321U));

    queue.generation = ~0U;
    expect_true("exhaustion command submits",
                nvme_admin_submit(&queue, 400U, 20U, &second));
    cqe.command_id = second.command_id;
    cqe.sq_id = 0U;
    cqe.sq_head = 1U;
    cqe.status = 0U;
    expect_true("exhaustion completion accepted",
                nvme_admin_complete(&queue, &second, &cqe, NULL, NULL) ==
                NVME_ADMIN_COMPLETION_SUCCESS);
    expect_true("exhausted queue is retired", queue.generation == 0U);
    expect_true("retired queue rejects commands",
                !nvme_admin_submit(&queue, 500U, 20U, &first));
}

int main(void)
{
    struct nvme_admin_cmd cmd;
    struct nvme_identify_info info;
    u8 identify[NVME_IDENTIFY_BYTES] = {0};

    expect_true("NVMe class", nvme_is_controller(1, 8, 2));
    expect_true("class fallback", nvme_is_controller(1, 8, 0));
    expect_true("non-NVMe class rejected", !nvme_is_controller(1, 6, 2));
    expect_true("queue depth 2", nvme_queue_depth_valid(2));
    expect_true("queue depth 1024", nvme_queue_depth_valid(1024));
    expect_true("queue depth non-power rejected", !nvme_queue_depth_valid(3));
    expect_true("queue depth oversized rejected", !nvme_queue_depth_valid(2048));

    expect_true("identify alignment rejected",
                !nvme_identify_buffer_valid(0x1000ULL, 512));
    expect_true("identify address alignment rejected",
                !nvme_identify_buffer_valid(0x1001ULL, NVME_IDENTIFY_BYTES));
    expect_true("identify buffer accepted",
                nvme_identify_buffer_valid(0x1000ULL, NVME_IDENTIFY_BYTES));
    expect_true("identify command builds",
                nvme_build_identify_controller(&cmd, 0x1000ULL,
                                               NVME_IDENTIFY_BYTES));
    expect_true("identify opcode", cmd.cdw0 == NVME_ADMIN_IDENTIFY);
    expect_true("identify PRP1", cmd.prp1 == 0x1000ULL);
    expect_true("identify CNS", cmd.cdw10 == NVME_IDENTIFY_CONTROLLER);
    expect_true("bad identify command rejected",
                !nvme_build_identify_controller(&cmd, 0x2000ULL, 512));

    identify[0] = 0x34; identify[1] = 0x12;
    identify[2] = 0x78; identify[3] = 0x56;
    identify[4] = 'S'; identify[5] = 'N'; identify[6] = ' '; identify[7] = ' ';
    identify[24] = 'M'; identify[25] = '2'; identify[26] = ' ';
    identify[64] = 'F'; identify[65] = '1'; identify[66] = ' ';
    identify[73] = 0xAA; identify[74] = 0xBB; identify[75] = 0xCC;
    identify[77] = 5;
    identify[80] = 0x03; identify[81] = 0x00; identify[82] = 0x01;
    identify[516] = 2;
    expect_true("identify response parses",
                nvme_identify_parse(identify, sizeof(identify), &info));
    expect_true("identify VID", info.vendor_id == 0x1234);
    expect_true("identify SSVID", info.subsystem_vendor_id == 0x5678);
    expect_true("identify serial trim", info.serial[0] == 'S' &&
                info.serial[1] == 'N' && info.serial[2] == 0);
    expect_true("identify model trim", info.model[0] == 'M' &&
                info.model[1] == '2' && info.model[2] == 0);
    expect_true("identify firmware trim", info.firmware[0] == 'F' &&
                info.firmware[1] == '1' && info.firmware[2] == 0);
    expect_true("identify OUI", info.ieee_oui[0] == 0xAA &&
                info.ieee_oui[2] == 0xCC);
    expect_true("identify namespace count", info.namespace_count == 2);
    expect_true("truncated response rejected",
                !nvme_identify_parse(identify, 519, &info));

    expect_true("status success phase zero", nvme_status_success(0));
    expect_true("status success phase one", nvme_status_success(1));
    expect_true("status code", nvme_status_code(0x0006) == 3);
    expect_true("status type", nvme_status_type(0x0200) == 1);
    test_admin_queue();
    test_namespace_geometry_and_prp();
    test_io_contract();

    if (failures) {
        printf("nvme: %d failures\n", failures);
        return 1;
    }
    printf("nvme: all checks passed\n");
    return 0;
}
