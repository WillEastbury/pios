#include <stdio.h>
#include "nvme.h"

static int failures;

static void expect_true(const char *name, bool value)
{
    if (!value) {
        printf("FAIL %s\n", name);
        failures++;
    }
}

static void test_admin_queue(void)
{
    struct nvme_admin_queue queue;
    struct nvme_admin_handle first;
    struct nvme_admin_handle second;
    struct nvme_admin_cqe cqe = {0};
    u32 result = 0U;
    u16 status = 0U;

    expect_true("invalid queue depth rejected",
                !nvme_admin_queue_init(&queue, 3U));
    expect_true("admin queue initialises", nvme_admin_queue_init(&queue, 16U));
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

    if (failures) {
        printf("nvme: %d failures\n", failures);
        return 1;
    }
    printf("nvme: all checks passed\n");
    return 0;
}
