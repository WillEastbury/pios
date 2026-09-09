#include <stdio.h>
#include <string.h>

#include "types.h"
#include "../src/usb_storage.c"

static int failures;

#define CHECK(expr) do { \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

#define MOCK_BULK_IN  0x81U
#define MOCK_BULK_OUT 0x02U
#define MOCK_MAX_CMDS 8U

struct mock_usb {
    u8 capacity10[8];
    u8 capacity16[SCSI_READ_CAPACITY16_LEN];
    u32 capacity10_actual;
    u32 capacity16_actual;
    u8 capacity16_csw_status;
    u32 command_count;
    u8 opcode[MOCK_MAX_CMDS];
    u8 cdb_len[MOCK_MAX_CMDS];
    u8 cdb[MOCK_MAX_CMDS][16];
    u32 transfer_len[MOCK_MAX_CMDS];
    u32 current_tag;
    u32 current_command;
    u8 phase;
    bool protocol_error;
};

static struct mock_usb mock;

static void store_be32(u8 *p, u32 value)
{
    p[0] = (u8)(value >> 24);
    p[1] = (u8)(value >> 16);
    p[2] = (u8)(value >> 8);
    p[3] = (u8)value;
}

static void store_be64(u8 *p, u64 value)
{
    store_be32(p, (u32)(value >> 32));
    store_be32(p + 4, (u32)value);
}

static void mock_reset(u32 last_lba10, u32 block_size10,
                       u64 last_lba16, u32 block_size16)
{
    memset(&mock, 0, sizeof(mock));
    store_be32(mock.capacity10, last_lba10);
    store_be32(mock.capacity10 + 4, block_size10);
    mock.capacity10_actual = sizeof(mock.capacity10);
    store_be64(mock.capacity16, last_lba16);
    store_be32(mock.capacity16 + 8, block_size16);
    mock.capacity16_actual = sizeof(mock.capacity16);
}

static struct usb_device mock_device(void)
{
    struct usb_device dev;

    memset(&dev, 0, sizeof(dev));
    dev.num_eps = 2U;
    dev.eps[0].address = MOCK_BULK_IN;
    dev.eps[0].attributes = USB_EP_ATTR_BULK;
    dev.eps[0].max_packet = 512U;
    dev.eps[1].address = MOCK_BULK_OUT;
    dev.eps[1].attributes = USB_EP_ATTR_BULK;
    dev.eps[1].max_packet = 512U;
    return dev;
}

void usb_register_driver(struct usb_driver *drv)
{
    (void)drv;
}

bool usb_bulk_msg(struct usb_device *dev, u8 ep_addr,
                  void *data, u32 len, u32 *actual)
{
    struct usb_cbw *cbw;
    struct usb_csw *csw;
    const u8 *response;
    u32 response_len;
    u8 opcode;

    (void)dev;
    if (!actual) {
        mock.protocol_error = true;
        return false;
    }

    if (ep_addr == MOCK_BULK_OUT && len == sizeof(struct usb_cbw)) {
        cbw = (struct usb_cbw *)data;
        if (mock.phase != 0U || mock.command_count >= MOCK_MAX_CMDS) {
            mock.protocol_error = true;
            return false;
        }
        mock.current_command = mock.command_count;
        mock.opcode[mock.command_count] = cbw->CBWCB[0];
        mock.cdb_len[mock.command_count] = cbw->bCBWCBLength;
        memcpy(mock.cdb[mock.command_count], cbw->CBWCB, sizeof(cbw->CBWCB));
        mock.transfer_len[mock.command_count] = cbw->dCBWDataTransferLength;
        mock.command_count++;
        mock.current_tag = cbw->dCBWTag;
        mock.phase = cbw->dCBWDataTransferLength ? 1U : 2U;
        *actual = len;
        return true;
    }

    if (ep_addr == MOCK_BULK_IN && mock.phase == 1U) {
        opcode = mock.opcode[mock.current_command];
        response = NULL;
        response_len = 0U;
        if (opcode == 0x12U) {
            static const u8 inquiry[36];
            response = inquiry;
            response_len = sizeof(inquiry);
        } else if (opcode == 0x25U) {
            response = mock.capacity10;
            response_len = mock.capacity10_actual;
        } else if (opcode == 0x9EU) {
            response = mock.capacity16;
            response_len = mock.capacity16_actual;
        } else {
            mock.protocol_error = true;
            return false;
        }
        if (response_len > len) {
            mock.protocol_error = true;
            return false;
        }
        memcpy(data, response, response_len);
        *actual = response_len;
        mock.phase = 2U;
        return true;
    }

    if (ep_addr == MOCK_BULK_IN && mock.phase == 2U &&
        len == sizeof(struct usb_csw)) {
        csw = (struct usb_csw *)data;
        opcode = mock.opcode[mock.current_command];
        csw->dCSWSignature = CSW_SIGNATURE;
        csw->dCSWTag = mock.current_tag;
        csw->dCSWDataResidue = 0U;
        csw->bCSWStatus = opcode == 0x9EU ? mock.capacity16_csw_status : 0U;
        *actual = len;
        mock.phase = 0U;
        return true;
    }

    mock.protocol_error = true;
    return false;
}

void uart_puts(const char *s) { (void)s; }
void uart_putc(char c) { (void)c; }
void uart_hex(u64 value) { (void)value; }
void timer_delay_ms(u64 ms) { (void)ms; }

static void test_capacity10_only(void)
{
    struct usb_device dev = mock_device();

    mock_reset(0x00123456U, 512U, 0U, 0U);
    CHECK(stor_probe(&dev));
    CHECK(!mock.protocol_error);
    CHECK(mock.command_count == 3U);
    CHECK(mock.opcode[0] == 0x12U);
    CHECK(mock.opcode[1] == 0x00U);
    CHECK(mock.opcode[2] == 0x25U);
    CHECK(mock.cdb_len[2] == 10U);
    CHECK(mock.transfer_len[2] == 8U);
    CHECK(usb_storage_block_size() == 512U);
    CHECK(usb_storage_num_blocks() == 0x00123457ULL);
    stor_disconnect(&dev);
}

static void test_capacity16_sentinel(void)
{
    struct usb_device dev = mock_device();

    mock_reset(0xFFFFFFFFU, 0U, 0x0000000100000000ULL, 4096U);
    CHECK(stor_probe(&dev));
    CHECK(!mock.protocol_error);
    CHECK(mock.command_count == 4U);
    CHECK(mock.opcode[2] == 0x25U);
    CHECK(mock.opcode[3] == 0x9EU);
    CHECK(mock.cdb_len[3] == 16U);
    CHECK(mock.cdb[3][1] == 0x10U);
    CHECK(mock.transfer_len[3] == SCSI_READ_CAPACITY16_LEN);
    for (u32 i = 2U; i < 10U; i++)
        CHECK(mock.cdb[3][i] == 0U);
    CHECK(mock.cdb[3][10] == 0U && mock.cdb[3][11] == 0U &&
          mock.cdb[3][12] == 0U &&
          mock.cdb[3][13] == SCSI_READ_CAPACITY16_LEN &&
          mock.cdb[3][14] == 0U && mock.cdb[3][15] == 0U);
    CHECK(usb_storage_block_size() == 4096U);
    CHECK(usb_storage_num_blocks() == 0x0000000100000001ULL);
    stor_disconnect(&dev);
}

static void test_capacity16_fail_closed(void)
{
    struct usb_device dev = mock_device();

    mock_reset(0xFFFFFFFFU, 512U, 0x0000000100000000ULL, 512U);
    mock.capacity16[12] = 0x80U;
    CHECK(!stor_probe(&dev));
    CHECK(mock.command_count == 4U);
    CHECK(!usb_storage_ready());
    CHECK(usb_storage_block_size() == 0U);
    CHECK(usb_storage_num_blocks() == 0ULL);

    mock_reset(0xFFFFFFFFU, 512U, 0x0000000100000000ULL, 512U);
    mock.capacity16_actual = SCSI_READ_CAPACITY16_LEN - 1U;
    CHECK(!stor_probe(&dev));
    CHECK(mock.command_count == 4U);
    CHECK(!usb_storage_ready());
    CHECK(usb_storage_block_size() == 0U);
    CHECK(usb_storage_num_blocks() == 0ULL);

    mock_reset(0xFFFFFFFFU, 512U, ~0ULL, 512U);
    CHECK(!stor_probe(&dev));
    CHECK(mock.command_count == 4U);
    CHECK(!usb_storage_ready());
    CHECK(usb_storage_num_blocks() == 0ULL);

    mock_reset(0xFFFFFFFFU, 512U, 0x0000000100000000ULL, 512U);
    mock.capacity16_csw_status = 1U;
    CHECK(!stor_probe(&dev));
    CHECK(mock.command_count == 4U);
    CHECK(!usb_storage_ready());
}

static void test_lba10_boundary(void)
{
    CHECK(!scsi_rw_needs_16(SCSI_LBA10_MAX, 1U));
    CHECK(scsi_rw_needs_16(SCSI_LBA10_MAX, 2U));
    CHECK(scsi_rw_needs_16(SCSI_LBA10_MAX + 1ULL, 1U));
    CHECK(scsi_rw_needs_16(0U, 0U));
}

int main(void)
{
    test_capacity10_only();
    test_capacity16_sentinel();
    test_capacity16_fail_closed();
    test_lba10_boundary();

    if (failures) {
        printf("usb_storage: %d failure(s)\n", failures);
        return 1;
    }
    printf("usb_storage: all checks passed\n");
    return 0;
}
