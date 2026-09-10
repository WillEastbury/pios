#include <stdio.h>
#include <string.h>

#include "types.h"
#include "media_engine_contract.h"
#include "pisp_be_contract.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static void put_le32(u8 *out, u32 value)
{
    out[0] = (u8)value;
    out[1] = (u8)(value >> 8);
    out[2] = (u8)(value >> 16);
    out[3] = (u8)(value >> 24);
}

static void make_config(u8 config[PISP_BE_CONFIG_BYTES])
{
    memset(config, 0, PISP_BE_CONFIG_BYTES);
    config[0] = 0xA5U;
    put_le32(config + PISP_BE_CONFIG_NUM_TILES_OFFSET, 1U);
    put_le32(config + PISP_BE_CONFIG_GLOBAL_BAYER_OFFSET,
             PISP_BE_CONFIG_ROOT_BIT);
}

static void make_spans(struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS])
{
    memset(spans, 0, sizeof(struct pisp_be_dma_span) * PISP_BE_MAX_SPANS);
    spans[0].allocation_id = 1U;
    spans[0].dma_allocation_base = 0x10000U;
    spans[0].allocation_capacity = 0x10000U;
    spans[0].allocation_offset = 0U;
    spans[0].window_capacity = 0x5000U;
    spans[0].used = PISP_BE_CONFIG_BYTES;
    spans[0].allocation_generation = 1U;
    spans[0].access = PISP_BE_DEVICE_READ;
    spans[0].logical_role = PISP_BE_SPAN_CONFIG;

    spans[1].allocation_id = 2U;
    spans[1].dma_allocation_base = 0x30000U;
    spans[1].allocation_capacity = 0x1000U;
    spans[1].allocation_offset = 0U;
    spans[1].window_capacity = 0x400U;
    spans[1].used = 0x200U;
    spans[1].allocation_generation = 2U;
    spans[1].access = PISP_BE_DEVICE_READ;
    spans[1].logical_role = PISP_BE_SPAN_MAIN_INPUT;

    spans[2].allocation_id = 3U;
    spans[2].dma_allocation_base = 0x40000U;
    spans[2].allocation_capacity = 0x1000U;
    spans[2].allocation_offset = 0U;
    spans[2].window_capacity = 0x400U;
    spans[2].used = 0x200U;
    spans[2].output_canary = 0xBEEFA11CU;
    spans[2].allocation_generation = 3U;
    spans[2].access = PISP_BE_DEVICE_WRITE;
    spans[2].logical_role = PISP_BE_SPAN_OUTPUT;
}

static void init_engine(struct media_engine_controller *controller,
                        struct media_engine_lease *lease, u32 controller_id)
{
    memset(controller, 0, sizeof(*controller));
    CHECK(media_engine_controller_init(controller, controller_id));
    CHECK(media_engine_passive_identify(controller, MEDIA_ENGINE_PISP_BE,
                                        0x02252700U));
    CHECK(media_engine_lease_acquire(controller, MEDIA_ENGINE_PISP_BE, lease));
    CHECK(media_engine_lease_active_for(controller, lease, MEDIA_ENGINE_PISP_BE));
}

static bool prepare(struct pisp_be_contract *contract,
                    struct media_engine_controller *controller,
                    const struct media_engine_lease *lease,
                    u8 config[PISP_BE_CONFIG_BYTES],
                    struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS],
                    u32 count, struct pisp_be_job_handle *handle)
{
    struct pisp_be_request request = {
        .layout_version = PISP_BE_REQUEST_LAYOUT_V1,
        .config_bytes = config,
        .config_bytes_len = PISP_BE_CONFIG_BYTES,
        ._reserved = 0U,
    };

    return pisp_be_contract_prepare(contract, controller, lease, &request,
                                    spans, count, handle);
}

static void init_contract(struct pisp_be_contract *contract, u32 controller_id)
{
    memset(contract, 0, sizeof(*contract));
    CHECK(pisp_be_contract_init(contract, controller_id));
}

static void test_abi_facts(void)
{
    struct pisp_be_contract contract;

    CHECK(PISP_BE_REQUEST_LAYOUT_V1 == 1U);
    CHECK(PISP_BE_CONFIG_BYTES == 16720U);
    CHECK(PISP_BE_CONFIG_INLINE_TILES_OFFSET == 6476U);
    CHECK(PISP_BE_CONFIG_INLINE_TILE_COUNT == 64U);
    CHECK(PISP_BE_CONFIG_INLINE_TILE_BYTES == 160U);
    CHECK(PISP_BE_CONFIG_NUM_TILES_OFFSET == 16716U);
    CHECK(PISP_BE_CONFIG_NUM_TILES_OFFSET + 4U == PISP_BE_CONFIG_BYTES);
    CHECK(PISP_BE_CONFIG_INLINE_TILES_OFFSET +
          PISP_BE_CONFIG_INLINE_TILE_COUNT * PISP_BE_CONFIG_INLINE_TILE_BYTES ==
          PISP_BE_CONFIG_NUM_TILES_OFFSET);
    CHECK(PISP_BE_CONFIG_GLOBAL_BAYER_OFFSET == 112U);
    CHECK(PISP_BE_CONFIG_GLOBAL_RGB_OFFSET == 116U);
    CHECK(PISP_BE_CONFIG_ROOT_BIT == 1U);
    CHECK(PISP_BE_DMA_ADDRESS_BITS == 36U);
    CHECK(PISP_BE_DMA_ADDRESS_LIMIT == 0x1000000000ULL);
    CHECK(PISP_BE_MAX_SPANS == 14U);
    CHECK(PISP_BE_JOB_CAPACITY == 4U);
    CHECK(sizeof(struct pisp_be_job_control) == 64U);
    CHECK(((usize)&((struct pisp_be_contract *)0)->payloads -
           (usize)&((struct pisp_be_contract *)0)->jobs) ==
          PISP_BE_JOB_CAPACITY * 64U);
    memset(&contract, 0, sizeof(contract));
    CHECK(!pisp_be_contract_init(NULL, 1U));
    CHECK(!pisp_be_contract_init(&contract, 0U));
}

static void test_config_validation(void)
{
    struct pisp_be_contract contract;
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    struct pisp_be_job_handle handle;
    struct pisp_be_request request;
    struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS];
    u8 config[PISP_BE_CONFIG_BYTES];

    init_contract(&contract, 44U);
    init_engine(&controller, &lease, 44U);
    make_config(config);
    make_spans(spans);
    request.layout_version = PISP_BE_REQUEST_LAYOUT_V1;
    request.config_bytes = config;
    request.config_bytes_len = PISP_BE_CONFIG_BYTES;
    request._reserved = 0U;
    CHECK(pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                   spans, 3U, &handle));
    CHECK(pisp_be_contract_abort(&contract, &controller, &lease, &handle));
    CHECK(pisp_be_contract_release(&contract, &controller, &lease, &handle));

    request.config_bytes_len = PISP_BE_CONFIG_BYTES - 1U;
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    request.config_bytes_len = PISP_BE_CONFIG_BYTES + 1U;
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    request.config_bytes_len = PISP_BE_CONFIG_BYTES;
    request.config_bytes = NULL;
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    request.config_bytes = config;
    request.layout_version = 0U;
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    request.layout_version = 2U;
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    request.layout_version = PISP_BE_REQUEST_LAYOUT_V1;
    request._reserved = 1U;
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    request._reserved = 0U;

    put_le32(config + PISP_BE_CONFIG_NUM_TILES_OFFSET, 0U);
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    put_le32(config + PISP_BE_CONFIG_NUM_TILES_OFFSET, 65U);
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    put_le32(config + PISP_BE_CONFIG_NUM_TILES_OFFSET, 64U);
    put_le32(config + PISP_BE_CONFIG_GLOBAL_BAYER_OFFSET, 0U);
    put_le32(config + PISP_BE_CONFIG_GLOBAL_RGB_OFFSET, 0U);
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    put_le32(config + PISP_BE_CONFIG_GLOBAL_BAYER_OFFSET, 1U);
    put_le32(config + PISP_BE_CONFIG_GLOBAL_RGB_OFFSET, 1U);
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                    spans, 3U, &handle));
    put_le32(config + PISP_BE_CONFIG_GLOBAL_BAYER_OFFSET, 0U);
    put_le32(config + PISP_BE_CONFIG_GLOBAL_RGB_OFFSET, 1U);
    CHECK(pisp_be_contract_prepare(&contract, &controller, &lease, &request,
                                   spans, 3U, &handle));
    CHECK(pisp_be_contract_abort(&contract, &controller, &lease, &handle));
    CHECK(pisp_be_contract_release(&contract, &controller, &lease, &handle));
}

static void expect_bad_span(struct pisp_be_contract *contract,
                            struct media_engine_controller *controller,
                            const struct media_engine_lease *lease,
                            u8 config[PISP_BE_CONFIG_BYTES],
                            struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS],
                            u32 count)
{
    struct pisp_be_job_handle handle;

    CHECK(!prepare(contract, controller, lease, config, spans, count, &handle));
    CHECK(handle._token == 0U && handle._generation == 0U &&
          handle._controller_id == 0U);
}

static void test_span_validation(void)
{
    struct pisp_be_contract contract;
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS];
    u8 config[PISP_BE_CONFIG_BYTES];

    init_contract(&contract, 45U);
    init_engine(&controller, &lease, 45U);
    make_config(config);
    make_spans(spans);
    expect_bad_span(&contract, &controller, &lease, config, spans, 0U);
    expect_bad_span(&contract, &controller, &lease, config, spans,
                    PISP_BE_MAX_SPANS + 1U);
    CHECK(!pisp_be_contract_prepare(&contract, &controller, &lease, NULL,
                                    spans, 3U, NULL));

#define BAD_SPAN(index, field, value) do { \
    make_spans(spans); spans[index].field = (value); \
    expect_bad_span(&contract, &controller, &lease, config, spans, 3U); \
} while (0)
    BAD_SPAN(0, allocation_id, 0U);
    BAD_SPAN(0, allocation_generation, 0U);
    BAD_SPAN(0, dma_allocation_base, 0U);
    BAD_SPAN(0, allocation_capacity, 0U);
    BAD_SPAN(0, window_capacity, 0U);
    BAD_SPAN(0, used, 0U);
    BAD_SPAN(0, used, PISP_BE_CONFIG_BYTES - 1U);
    BAD_SPAN(0, used, 0x6000U);
    BAD_SPAN(0, allocation_offset, 0x10000U);
    BAD_SPAN(0, window_capacity, 0x10001U);
    BAD_SPAN(0, dma_allocation_base, PISP_BE_DMA_ADDRESS_LIMIT);
    BAD_SPAN(0, dma_allocation_base, PISP_BE_DMA_ADDRESS_LIMIT - 0x8000U);
    BAD_SPAN(0, _reserved, 1U);
    BAD_SPAN(0, access, 0U);
    BAD_SPAN(0, access, 4U);
    BAD_SPAN(0, access, PISP_BE_DEVICE_WRITE);
    BAD_SPAN(0, output_canary, 1U);
    BAD_SPAN(0, dma_allocation_base, 0x10001U);
    BAD_SPAN(0, logical_role, PISP_BE_SPAN_INVALID);
    BAD_SPAN(1, access, PISP_BE_DEVICE_WRITE);
    BAD_SPAN(1, output_canary, 1U);
    BAD_SPAN(1, dma_allocation_base, 0x30002U);
    BAD_SPAN(1, logical_role, PISP_BE_SPAN_AUX_INPUT);
    BAD_SPAN(2, access, PISP_BE_DEVICE_READ);
    BAD_SPAN(2, output_canary, 0U);
    BAD_SPAN(2, dma_allocation_base, 0x40004U);
#undef BAD_SPAN

    make_spans(spans);
    spans[0].logical_role = PISP_BE_SPAN_AUX_INPUT;
    expect_bad_span(&contract, &controller, &lease, config, spans, 3U);
    make_spans(spans);
    spans[1].logical_role = PISP_BE_SPAN_CONFIG;
    expect_bad_span(&contract, &controller, &lease, config, spans, 3U);
    make_spans(spans);
    spans[3] = spans[0];
    spans[3].allocation_id = 4U;
    spans[3].allocation_generation = 4U;
    spans[3].dma_allocation_base = 0x60000U;
    expect_bad_span(&contract, &controller, &lease, config, spans, 4U);
    make_spans(spans);
    spans[2].logical_role = PISP_BE_SPAN_AUX_INPUT;
    spans[2].access = PISP_BE_DEVICE_READ;
    spans[2].output_canary = 0U;
    expect_bad_span(&contract, &controller, &lease, config, spans, 3U);
    make_spans(spans);
    spans[2].dma_allocation_base = spans[1].dma_allocation_base;
    expect_bad_span(&contract, &controller, &lease, config, spans, 3U);
    make_spans(spans);
    spans[1].allocation_id = spans[0].allocation_id;
    spans[1].allocation_generation = spans[0].allocation_generation;
    expect_bad_span(&contract, &controller, &lease, config, spans, 3U);
    make_spans(spans);
    spans[1].allocation_id = spans[0].allocation_id;
    spans[1].allocation_generation = spans[0].allocation_generation;
    spans[1].dma_allocation_base = spans[0].dma_allocation_base;
    spans[1].allocation_capacity = spans[0].allocation_capacity;
    spans[1].allocation_offset = 0x6000U;
    spans[1].window_capacity = 0x400U;
    spans[1].used = 0x200U;
    CHECK(prepare(&contract, &controller, &lease, config, spans, 3U,
                  &(struct pisp_be_job_handle){0}));
}

static void test_lease_binding_and_lifecycle(void)
{
    struct pisp_be_contract contract, other_contract;
    struct media_engine_controller controller, other_controller;
    struct media_engine_lease lease, hevc_lease, stale_lease;
    struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS];
    struct pisp_be_job_handle handle, stale_handle, forged;
    struct pisp_be_output_canary report;
    enum pisp_be_job_state state;
    u8 byte;
    u8 config[PISP_BE_CONFIG_BYTES];

    init_contract(&contract, 46U);
    init_contract(&other_contract, 47U);
    init_engine(&controller, &lease, 46U);
    make_config(config);
    make_spans(spans);
    CHECK(!prepare(&other_contract, &controller, &lease, config, spans, 3U,
                   &handle));
    memset(&other_controller, 0, sizeof(other_controller));
    CHECK(media_engine_controller_init(&other_controller, 47U));
    CHECK(media_engine_passive_identify(&other_controller, MEDIA_ENGINE_HEVC,
                                        0x202U));
    CHECK(media_engine_lease_acquire(&other_controller, MEDIA_ENGINE_HEVC,
                                     &hevc_lease));
    CHECK(!prepare(&contract, &other_controller, &hevc_lease, config, spans,
                   3U, &handle));
    hevc_lease = lease;
    hevc_lease._token = (hevc_lease._token & ~0xFFULL) |
                        (u64)MEDIA_ENGINE_HEVC;
    CHECK(!prepare(&contract, &controller, &hevc_lease, config, spans, 3U,
                   &handle));
    CHECK(prepare(&contract, &controller, &lease, config, spans, 3U, &handle));
    stale_handle = handle;
    CHECK(!pisp_be_contract_state_get(&other_contract, &handle, &state));
    CHECK(pisp_be_contract_state_get(&contract, &handle, &state));
    CHECK(state == PISP_BE_JOB_PREPARED);
    CHECK(!pisp_be_contract_complete(&contract, &controller, &lease, &handle));
    CHECK(!pisp_be_contract_release(&contract, &controller, &lease, &handle));
    CHECK(!pisp_be_contract_report_output_canaries(&contract, &controller,
          &lease, &handle, NULL, 0U));
    CHECK(pisp_be_contract_config_byte(&contract, &handle, 0U, &byte));
    CHECK(byte == 0xA5U);
    config[0] = 0x5AU;
    CHECK(pisp_be_contract_config_byte(&contract, &handle, 0U, &byte));
    CHECK(byte == 0xA5U);
    CHECK(!pisp_be_contract_config_byte(&contract, &handle,
                                        PISP_BE_CONFIG_BYTES, &byte));
    CHECK(pisp_be_contract_mark_in_flight(&contract, &controller, &lease,
                                          &handle));
    CHECK(!pisp_be_contract_mark_in_flight(&contract, &controller, &lease,
                                           &handle));
    CHECK(pisp_be_contract_state_get(&contract, &handle, &state));
    CHECK(state == PISP_BE_JOB_IN_FLIGHT);
    CHECK(pisp_be_contract_complete(&contract, &controller, &lease, &handle));
    CHECK(!pisp_be_contract_complete(&contract, &controller, &lease, &handle));
    report.span_index = 2U;
    report._reserved = 0U;
    report.observed_canary = 0U;
    CHECK(!pisp_be_contract_report_output_canaries(&contract, &controller,
          &lease, &handle, &report, 1U));
    CHECK(!pisp_be_contract_release(&contract, &controller, &lease, &handle));
    report.observed_canary = spans[2].output_canary;
    CHECK(pisp_be_contract_report_output_canaries(&contract, &controller,
          &lease, &handle, &report, 1U));
    CHECK(!pisp_be_contract_report_output_canaries(&contract, &controller,
          &lease, &handle, &report, 1U));
    CHECK(pisp_be_contract_release(&contract, &controller, &lease, &handle));
    CHECK(!pisp_be_contract_state_get(&contract, &stale_handle, &state));
    CHECK(!pisp_be_contract_config_byte(&contract, &stale_handle, 0U, &byte));
    CHECK(!pisp_be_contract_release(&contract, &controller, &lease,
                                    &stale_handle));
    CHECK(prepare(&contract, &controller, &lease, config, spans, 3U, &handle));
    CHECK(handle._generation != stale_handle._generation);
    forged = handle;
    forged._token ^= 0x100ULL;
    CHECK(!pisp_be_contract_abort(&contract, &controller, &lease, &forged));
    forged = handle;
    forged._token ^= 0x1000000000000000ULL;
    CHECK(!pisp_be_contract_abort(&contract, &controller, &lease, &forged));
    forged = handle;
    forged._controller_id++;
    CHECK(!pisp_be_contract_abort(&contract, &controller, &lease, &forged));
    CHECK(pisp_be_contract_abort(&contract, &controller, &lease, &handle));
    CHECK(pisp_be_contract_release(&contract, &controller, &lease, &handle));

    CHECK(prepare(&contract, &controller, &lease, config, spans, 3U, &handle));
    stale_lease = lease;
    CHECK(media_engine_lease_abort(&controller, &lease));
    CHECK(!pisp_be_contract_abort(&contract, &controller, &stale_lease,
                                  &handle));
    CHECK(!pisp_be_contract_mark_in_flight(&contract, &controller, &stale_lease,
                                           &handle));
}

static void test_canary_and_pool(void)
{
    struct pisp_be_contract contract;
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS];
    struct pisp_be_job_handle handles[PISP_BE_JOB_CAPACITY], handle;
    struct pisp_be_output_canary reports[2];
    u8 config[PISP_BE_CONFIG_BYTES];
    u32 i;

    init_contract(&contract, 48U);
    init_engine(&controller, &lease, 48U);
    make_config(config);
    make_spans(spans);
    spans[3] = spans[2];
    spans[3].allocation_id = 4U;
    spans[3].allocation_generation = 4U;
    spans[3].dma_allocation_base = 0x50000U;
    spans[3].output_canary = 0xABCDEF11U;
    CHECK(prepare(&contract, &controller, &lease, config, spans, 4U, &handle));
    CHECK(pisp_be_contract_mark_in_flight(&contract, &controller, &lease,
                                          &handle));
    CHECK(pisp_be_contract_complete(&contract, &controller, &lease, &handle));
    reports[0] = (struct pisp_be_output_canary){ 2U, 0U,
                                                  spans[2].output_canary };
    reports[1] = (struct pisp_be_output_canary){ 2U, 0U,
                                                  spans[2].output_canary };
    CHECK(!pisp_be_contract_report_output_canaries(&contract, &controller,
          &lease, &handle, reports, 1U));
    CHECK(!pisp_be_contract_report_output_canaries(&contract, &controller,
          &lease, &handle, reports, 2U));
    reports[1].span_index = 3U;
    reports[1].observed_canary = spans[3].output_canary;
    CHECK(pisp_be_contract_report_output_canaries(&contract, &controller,
          &lease, &handle, reports, 2U));
    CHECK(pisp_be_contract_release(&contract, &controller, &lease, &handle));

    for (i = 0U; i < PISP_BE_JOB_CAPACITY; i++)
        CHECK(prepare(&contract, &controller, &lease, config, spans, 3U,
                      &handles[i]));
    CHECK(!prepare(&contract, &controller, &lease, config, spans, 3U, &handle));
    for (i = 0U; i < PISP_BE_JOB_CAPACITY; i++) {
        CHECK(pisp_be_contract_abort(&contract, &controller, &lease,
                                     &handles[i]));
        CHECK(pisp_be_contract_release(&contract, &controller, &lease,
                                       &handles[i]));
    }
}

static void test_full_generation_wrap(void)
{
    struct pisp_be_contract contract;
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS];
    struct pisp_be_job_handle wrapped, stale, reused;
    u8 config[PISP_BE_CONFIG_BYTES];

    init_contract(&contract, 49U);
    init_engine(&controller, &lease, 49U);
    make_config(config);
    make_spans(spans);
    contract.jobs[0].generation = ~0ULL;
    CHECK(prepare(&contract, &controller, &lease, config, spans, 3U,
                  &wrapped));
    CHECK(wrapped._generation == ~0ULL);
    stale = wrapped;
    CHECK(pisp_be_contract_abort(&contract, &controller, &lease, &wrapped));
    CHECK(pisp_be_contract_release(&contract, &controller, &lease, &wrapped));
    CHECK(prepare(&contract, &controller, &lease, config, spans, 3U,
                  &reused));
    CHECK(reused._generation == 1U);
    CHECK(!pisp_be_contract_abort(&contract, &controller, &lease, &stale));
    CHECK(pisp_be_contract_abort(&contract, &controller, &lease, &reused));
    CHECK(pisp_be_contract_release(&contract, &controller, &lease, &reused));
}

int main(void)
{
    test_abi_facts();
    test_config_validation();
    test_span_validation();
    test_lease_binding_and_lifecycle();
    test_canary_and_pool();
    test_full_generation_wrap();

    if (failures) {
        printf("pisp be contract: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("pisp be contract: %d checks passed\n", checks);
    return checks > 100 ? 0 : 1;
}
