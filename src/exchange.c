/* SD adapter for the optional pre-created PIOSXFER partition. */
#include "types.h"
#include "platform.h"
#include "exchange.h"
#include "sd.h"
#include "uart.h"

static struct exchange_service g_exchange ALIGNED(64);

static bool exchange_sd_read(void *context, u32 lba, u8 out[512])
{
    (void)context;
    return sd_read_block(lba, out);
}

static bool exchange_sd_write(void *context, u32 lba, const u8 in[512])
{
    (void)context;
    return sd_write_block(lba, in);
}

void exchange_init(void)
{
    u8 mbr[STORAGE_LAYOUT_MBR_BYTES] ALIGNED(64);
    const sd_card_t *card = sd_get_card_info();
    struct exchange_service_backend backend = {
        .read = exchange_sd_read,
        .write = exchange_sd_write,
        .context = NULL,
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
        .writable = true,
#else
        .writable = false,
#endif
    };
    enum exchange_service_result result;

#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    if (!sd_qemu_virtio_blk_ready()) {
        (void)exchange_service_init(&g_exchange, mbr, 0U, &backend);
        uart_puts("[xfer] unavailable (no virtio block)\n");
        return;
    }
#endif
    if (!card || card->capacity == 0U ||
        (card->capacity % SD_BLOCK_SIZE) != 0U ||
        !sd_read_block(0U, mbr)) {
        (void)exchange_service_init(&g_exchange, mbr, 0U, &backend);
        uart_puts("[xfer] unavailable (MBR)\n");
        return;
    }
    result = exchange_service_init(&g_exchange, mbr,
                                   card->capacity / SD_BLOCK_SIZE, &backend);
    uart_puts("[xfer] ");
    uart_puts(exchange_service_result_name(result));
    if (result == EXCHANGE_SERVICE_OK)
        uart_puts(backend.writable ? " mounted rw\n" : " mounted ro\n");
    else
        uart_puts("\n");
}

void exchange_status(struct exchange_service_status *out)
{
    exchange_service_status(&g_exchange, out);
}

struct exchange_service *exchange_runtime_service(void)
{
    return &g_exchange;
}
