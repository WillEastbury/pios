#include "types.h"
#include "b50_native.h"
#include "pcie1.h"
#include "pcie1_bar_lease.h"
#include "pcie1_containment.h"
#include "lzero.h"
#include "mmu.h"
#ifndef B50_NATIVE_HOST_TEST
#include "mmio.h"
#else
u32 b50_native_test_read(u64 addr);
void b50_native_test_write(u64 addr, u32 value);
u64 b50_native_test_read64(u64 addr);
void b50_native_test_write64(u64 addr, u64 value);
void *b50_native_test_pointer(u64 addr);
#define mmio_read b50_native_test_read
#define mmio_write b50_native_test_write
#define mmio_read64 b50_native_test_read64
#define mmio_write64 b50_native_test_write64
#endif
#include "timer.h"
#include "gic.h"
#include "exception.h"
#include "uart.h"
#include "bmg_guc.h"
#include "adrv.h"
#include "intel_xe2_guc.h"
#include "intel_guc_boot.h"
#include "intel_guc_ct.h"
#include "intel_bmg_submit.h"
#include "module.h"

/* All lifecycle mutation is core-0 scheduled context, never IRQ context. */
#if PIOS_HAS_PCIE1
static struct pcie1_bar_lease lease;
static struct pcie1_containment containment;
static struct {
    struct pcie1_bar_lease_handle bar;
    struct pcie1_containment_endpoint_handle endpoint;
} ALIGNED(64) authority;
static struct {
    u32 initialized;
    u32 attached;
    u32 firmware_handle;
    u32 firmware_offset;
    u32 firmware_state;
    u32 firmware_failure;
    u32 forcewake_handle;
    u32 forcewake_state;
    u32 forcewake_gmd_id;
    u32 forcewake_failure;
    u64 forcewake_deadline;
    u32 ggtt_handle;
    u32 ggtt_state;
    u32 ggtt_page;
    u32 ggtt_failure;
} ALIGNED(64) native;
static struct bmg_guc_artifact guc_artifact ALIGNED(64);
static struct {
    u32 handle;
    u32 state;
    u32 offset;
    u32 failure;
    u32 revision;
    u8 _pad[44];
} ALIGNED(64) boot_prep;
static struct intel_guc_boot_layout boot_layout ALIGNED(64);
static u32 validation_stage;
static struct b50_native_diag ggtt_diag ALIGNED(64);
static struct b50_native_boot_diag boot_diag ALIGNED(64);
static struct {
    u64 deadline;
    u32 handle;
    u32 state;
    u32 offset;
    u32 index;
    u32 retries;
    u32 request_len;
    u32 fence;
    u32 response;
    u32 failure;
    u32 request[4];
    u8 _pad[8];
} ALIGNED(64) ct_boot;
static struct {
    u64 deadline;
    u32 handle;
    u32 state;
    u32 fence;
    u32 response;
    u32 failure;
    u32 ring_tail;
    u32 descriptor_lo;
    u32 descriptor_hi;
    u8 _pad[24];
} ALIGNED(64) context_boot;
static struct {
    u64 deadline;
    u32 handle;
    u32 state;
    u32 response;
    u32 failure;
    u32 result;
    u32 completion;
    u32 fence;
    u32 mode_done;
    u8 _pad[24];
} ALIGNED(64) submit_boot;
static u32 submit_engine_diag[8] ALIGNED(64);
static struct {
    u64 deadline;
    u32 handle;
    u32 state;
    u32 index;
    u32 last_status;
    u32 failure;
    u32 dma_ctrl;
    u32 ready;
    u8 _pad[24];
} ALIGNED(64) guc_boot;

#define B50_FW_STATE_IDLE       0U
#define B50_FW_STATE_COPY       1U
#define B50_FW_STATE_VERIFY     2U
#define B50_FW_STATE_STAGED     3U
#define B50_FW_CHUNK_BYTES      4096U
#define B50_FW_GUARD_BYTES      64U
#define B50_FW_PAYLOAD_OFFSET   4096U

#define B50_FORCEWAKE_GT         0x0A188U
#define B50_FORCEWAKE_ACK_GT     0x00DFCU
#define B50_FORCEWAKE_RENDER     0x0A278U
#define B50_FORCEWAKE_ACK_RENDER 0x00D84U
#define B50_GMD_ID               0x00D8CU
#define B50_FORCEWAKE_REQUEST    0x00010001U
#define B50_FORCEWAKE_RELEASE    0x00010000U
#define B50_FORCEWAKE_ACK        0x00000001U
#define B50_FORCEWAKE_TIMEOUT_MS 50U

#define B50_FWK_IDLE             0U
#define B50_FWK_WAIT_GT          1U
#define B50_FWK_WAIT_RENDER      2U
#define B50_FWK_WAIT_RENDER_OFF  3U
#define B50_FWK_WAIT_GT_OFF      4U
#define B50_FWK_DONE             5U

#define B50_GGTT_IDLE            0U
#define B50_GGTT_MAP_FW          1U
#define B50_GGTT_MAP_ADS         2U
#define B50_GGTT_MAP_QUEUE       3U
#define B50_GGTT_VERIFY_FW       4U
#define B50_GGTT_VERIFY_ADS      5U
#define B50_GGTT_VERIFY_QUEUE    6U
#define B50_GGTT_DONE            7U
#define B50_GGTT_PTES_PER_STEP   16U

#define B50_PREP_IDLE            0U
#define B50_PREP_ZERO_ADS        1U
#define B50_PREP_ZERO_LOG        2U
#define B50_PREP_BUILD           3U
#define B50_PREP_DONE            4U
#define B50_PREP_CHUNK_BYTES     4096U
#define B50_PREP_GUARD_BYTES     64U

#define B50_BOOT_IDLE            0U
#define B50_BOOT_ACQUIRE_GT      1U
#define B50_BOOT_WAIT_GT         2U
#define B50_BOOT_RESET           3U
#define B50_BOOT_WAIT_RESET      4U
#define B50_BOOT_PAT             5U
#define B50_BOOT_MOCS            6U
#define B50_BOOT_TLB_INV         7U
#define B50_BOOT_TLB_WAIT        8U
#define B50_BOOT_WOPCM           9U
#define B50_BOOT_PARAMS          10U
#define B50_BOOT_BME             11U
#define B50_BOOT_DMA_START       12U
#define B50_BOOT_DMA_WAIT        13U
#define B50_BOOT_UCODE_WAIT      14U
#define B50_BOOT_RELEASE_GT      15U
#define B50_BOOT_WAIT_GT_OFF     16U
#define B50_BOOT_DONE            17U

#define B50_GUC_STATUS           0x0C000U
#define B50_GUC_WOPCM_SIZE       0x0C050U
#define B50_GUC_SHIM_CONTROL     0x0C064U
#define B50_GUC_SOFT_SCRATCH     0x0C180U
#define B50_GUC_RSA_SCRATCH0     0x0C200U
#define B50_GUC_DMA_ADDR0_LO     0x0C300U
#define B50_GUC_DMA_ADDR0_HI     0x0C304U
#define B50_GUC_DMA_ADDR1_LO     0x0C308U
#define B50_GUC_DMA_ADDR1_HI     0x0C30CU
#define B50_GUC_DMA_COPY_SIZE    0x0C310U
#define B50_GUC_DMA_CTRL         0x0C314U
#define B50_GUC_WOPCM_OFFSET     0x0C340U
#define B50_GUC_TLB_INV_DESC0    0x0CF7CU
#define B50_GUC_TLB_INV_DESC1    0x0CF80U
#define B50_GDRST                0x0941CU
#define B50_GT_PM_CONFIG         0x13816CU
#define B50_PMINTRMSK            0x0A168U
#define B50_GUC_RESET_BIT        (1U << 3)
#define B50_GUC_MIA_RESET        1U
#define B50_GUC_DMA_START_BIT    1U
#define B50_GUC_DMA_START_VALUE  0x00110011U
#define B50_GUC_DMA_DISABLE      0x00100000U
#define B50_L2_BLOCK_SIZE        0x00200000U
#define B50_LIVE_VALIDATE_MS     250U

#define B50_CT_IDLE              0U
#define B50_CT_ZERO              1U
#define B50_CT_CFG_SEND          2U
#define B50_CT_CFG_WAIT          3U
#define B50_CT_CONTROL_SEND      4U
#define B50_CT_CONTROL_WAIT      5U
#define B50_CT_PROBE_SEND        6U
#define B50_CT_PROBE_WAIT        7U
#define B50_CT_DONE              8U
#define B50_CT_ZERO_CHUNK        4096U
#define B50_CT_BASE_OFFSET       0x00120000U
#define B50_CT_H2G_DESC_OFFSET   (B50_CT_BASE_OFFSET + 0x0000U)
#define B50_CT_G2H_DESC_OFFSET   (B50_CT_BASE_OFFSET + 0x0800U)
#define B50_CT_H2G_RING_OFFSET   (B50_CT_BASE_OFFSET + 0x1000U)
#define B50_CT_G2H_RING_OFFSET   (B50_CT_BASE_OFFSET + 0x2000U)
#define B50_CT_HWCONFIG_OFFSET   (B50_CT_BASE_OFFSET + 0x22000U)
#define B50_CT_TOTAL_BYTES       0x00023000U
#define B50_CT_MMIO_TIMEOUT_MS   50U
#define B50_CT_PROBE_TIMEOUT_MS  500U
#define B50_CT_MAX_RETRIES       2U
#define B50_VF_SW_FLAG0          0x00190240U
#define B50_GUC_HOST_INTERRUPT   0x001901F0U
#define B50_GUC_GET_HWCONFIG     0x4100U
#define B50_CONTEXT_IDLE         0U
#define B50_CONTEXT_BUILD        1U
#define B50_CONTEXT_REGISTER     2U
#define B50_CONTEXT_WAIT         3U
#define B50_CONTEXT_DONE         4U
#define B50_CONTEXT_GUC_ID       1U
#define B50_CONTEXT_LRC_OFFSET   0x00160000U
#define B50_CONTEXT_RESULT_OFFSET 0x00170000U
#define B50_CONTEXT_COMPLETION_OFFSET 0x00170004U
#define B50_CONTEXT_TIMEOUT_MS   500U
#define B50_SUBMIT_IDLE          0U
#define B50_SUBMIT_ENABLE_SEND   1U
#define B50_SUBMIT_ENABLE_WAIT   2U
#define B50_SUBMIT_SCHED_SEND    3U
#define B50_SUBMIT_WAIT          4U
#define B50_SUBMIT_DONE          5U
#define B50_SUBMIT_TIMEOUT_MS    500U
#define B50_GUC_SCHED_MODE_DONE  0x90001002U
#define B50_CCS0_RING_TAIL       0x001A030U
#define B50_CCS0_RING_HEAD       0x001A034U
#define B50_CCS0_RING_START      0x001A038U
#define B50_CCS0_RING_CTL        0x001A03CU
#define B50_CCS0_IPEHR           0x001A068U
#define B50_CCS0_ACTHD           0x001A074U
#define B50_CCS0_HWS_PGA         0x001A080U
#define B50_CCS0_CONTEXT_CONTROL 0x001A244U

#define B50_ROOT_MSI_INTID 256U
static struct {
    volatile u64 sequence;
    u64 deadline;
    u8 _pad[48];
} ALIGNED(64) trigger;
static struct {
    u64 acknowledged;
    u64 queued;
    u8 _pad[48];
} ALIGNED(64) continuation;

/* Decode remains disabled. Any interrupt before a real engine exists is
 * unexpected and can only revoke authority, never certify a completion. */
static void native_irq(void)
{
    gic_disable_irq(B50_ROOT_MSI_INTID);
    trigger.deadline = timer_monotonic_ms() + 100U;
    u64 sequence = trigger.sequence + 1U;
    if (!sequence) sequence = ~0ULL;
    dmb();
    trigger.sequence = sequence;
    (void)airq_post_from(0U, AIRQ_SRC_PCIE1_MSI, 0U);
}

static const char *quarantine(void);
static u64 b50_mmio_addr(u32 offset);

static void forcewake_drop(void)
{
    if (!native.attached)
        return;
    mmio_write(b50_mmio_addr(B50_FORCEWAKE_RENDER),
               B50_FORCEWAKE_RELEASE);
    mmio_write(b50_mmio_addr(B50_FORCEWAKE_GT), B50_FORCEWAKE_RELEASE);
    dsb();
    native.forcewake_state = B50_FWK_IDLE;
}

static u32 firmware_stage_step(void *ctx, u64 deadline_ms)
{
    u8 *region;
    u8 *payload;
    u32 remain, chunk;
    (void)ctx;
    if (!native.attached || !guc_artifact.data ||
        guc_artifact.bytes == 0U ||
        guc_artifact.bytes >
            PIOS_DMA_PCIE1_GUC_FW_SIZE - B50_FW_PAYLOAD_OFFSET -
            B50_FW_GUARD_BYTES)
        return ADRV_STEP_FAILED;
#ifndef B50_NATIVE_HOST_TEST
    region = (u8 *)(usize)PIOS_DMA_PCIE1_GUC_FW_BASE;
#else
    region = b50_native_test_pointer(PIOS_DMA_PCIE1_GUC_FW_BASE);
#endif
    payload = region + B50_FW_PAYLOAD_OFFSET;
    if (native.firmware_offset == 0U &&
        native.firmware_state == B50_FW_STATE_COPY) {
        for (u32 i = 0U; i < B50_FW_GUARD_BYTES; i++) {
            region[i] = 0xA5U;
            region[PIOS_DMA_PCIE1_GUC_FW_SIZE -
                   B50_FW_GUARD_BYTES + i] = 0x5AU;
        }

        dsb();
    }
    if (native.firmware_state == B50_FW_STATE_COPY) {
        remain = guc_artifact.bytes - native.firmware_offset;
        chunk = remain < B50_FW_CHUNK_BYTES ? remain : B50_FW_CHUNK_BYTES;
        memcpy(payload + native.firmware_offset,
               guc_artifact.data + native.firmware_offset, chunk);
        dsb();
        native.firmware_offset += chunk;
        if (native.firmware_offset == guc_artifact.bytes) {
            native.firmware_offset = 0U;
            native.firmware_state = B50_FW_STATE_VERIFY;
        }
        return timer_monotonic_ms() <= deadline_ms ?
            ADRV_STEP_PROGRESS : ADRV_STEP_FAILED;
    }
    if (native.firmware_state == B50_FW_STATE_VERIFY) {
        remain = guc_artifact.bytes - native.firmware_offset;
        chunk = remain < B50_FW_CHUNK_BYTES ? remain : B50_FW_CHUNK_BYTES;
        if (memcmp(payload + native.firmware_offset,
                   guc_artifact.data + native.firmware_offset, chunk) != 0)
            return ADRV_STEP_FAILED;
        native.firmware_offset += chunk;
        if (native.firmware_offset != guc_artifact.bytes)
            return timer_monotonic_ms() <= deadline_ms ?
                ADRV_STEP_PROGRESS : ADRV_STEP_FAILED;
        for (u32 i = 0U; i < B50_FW_GUARD_BYTES; i++)
            if (region[i] != 0xA5U ||
                region[PIOS_DMA_PCIE1_GUC_FW_SIZE -
                       B50_FW_GUARD_BYTES + i] != 0x5AU)
                return ADRV_STEP_FAILED;
        dsb();
        return ADRV_STEP_DONE;
    }
    return ADRV_STEP_FAILED;
}

static u64 b50_mmio_addr(u32 offset)
{
    return PIOS_PCIE1_CPU_WIN_BASE + offset;
}

static u32 forcewake_step(void *ctx, u64 deadline_ms)
{
    u64 now = timer_monotonic_ms();
    u32 ack;
    (void)ctx;
    if (!native.attached || now > deadline_ms)
        return ADRV_STEP_FAILED;
    switch (native.forcewake_state) {
    case B50_FWK_IDLE:
        mmio_write(b50_mmio_addr(B50_FORCEWAKE_GT), B50_FORCEWAKE_REQUEST);
        dsb();
        native.forcewake_deadline = now + B50_FORCEWAKE_TIMEOUT_MS;
        native.forcewake_state = B50_FWK_WAIT_GT;
        return ADRV_STEP_PROGRESS;
    case B50_FWK_WAIT_GT:
        ack = mmio_read(b50_mmio_addr(B50_FORCEWAKE_ACK_GT));
        if (ack == 0xFFFFFFFFU || now >= native.forcewake_deadline)
            return ADRV_STEP_FAILED;
        if (!(ack & B50_FORCEWAKE_ACK))
            return ADRV_STEP_IDLE;
        mmio_write(b50_mmio_addr(B50_FORCEWAKE_RENDER),
                   B50_FORCEWAKE_REQUEST);
        dsb();
        native.forcewake_deadline = now + B50_FORCEWAKE_TIMEOUT_MS;
        native.forcewake_state = B50_FWK_WAIT_RENDER;
        return ADRV_STEP_PROGRESS;
    case B50_FWK_WAIT_RENDER:
        ack = mmio_read(b50_mmio_addr(B50_FORCEWAKE_ACK_RENDER));
        if (ack == 0xFFFFFFFFU || now >= native.forcewake_deadline)
            return ADRV_STEP_FAILED;
        if (!(ack & B50_FORCEWAKE_ACK))
            return ADRV_STEP_IDLE;
        native.forcewake_gmd_id = mmio_read(b50_mmio_addr(B50_GMD_ID));
        if (native.forcewake_gmd_id != 0x05004000U)
            return ADRV_STEP_FAILED;
        mmio_write(b50_mmio_addr(B50_FORCEWAKE_RENDER),
                   B50_FORCEWAKE_RELEASE);
        dsb();
        native.forcewake_deadline = now + B50_FORCEWAKE_TIMEOUT_MS;
        native.forcewake_state = B50_FWK_WAIT_RENDER_OFF;
        return ADRV_STEP_PROGRESS;
    case B50_FWK_WAIT_RENDER_OFF:
        ack = mmio_read(b50_mmio_addr(B50_FORCEWAKE_ACK_RENDER));
        if (ack == 0xFFFFFFFFU || now >= native.forcewake_deadline)
            return ADRV_STEP_FAILED;
        if (ack & B50_FORCEWAKE_ACK)
            return ADRV_STEP_IDLE;
        mmio_write(b50_mmio_addr(B50_FORCEWAKE_GT), B50_FORCEWAKE_RELEASE);
        dsb();
        native.forcewake_deadline = now + B50_FORCEWAKE_TIMEOUT_MS;
        native.forcewake_state = B50_FWK_WAIT_GT_OFF;
        return ADRV_STEP_PROGRESS;
    case B50_FWK_WAIT_GT_OFF:
        ack = mmio_read(b50_mmio_addr(B50_FORCEWAKE_ACK_GT));
        if (ack == 0xFFFFFFFFU || now >= native.forcewake_deadline)
            return ADRV_STEP_FAILED;
        if (ack & B50_FORCEWAKE_ACK)
            return ADRV_STEP_IDLE;
        native.forcewake_state = B50_FWK_DONE;
        return ADRV_STEP_DONE;
    default:
        return ADRV_STEP_FAILED;
    }
}

static u32 ggtt_firmware_pages(void)
    {
        return (guc_artifact.bytes + INTEL_XE2_GGTT_PAGE_BYTES - 1U) /
               INTEL_XE2_GGTT_PAGE_BYTES;
    }

static bool ggtt_pte(u32 state, u32 page, u32 *offset_out, u64 *pte_out)
    {
        u32 ggtt;
        u64 dma;
        if (state == B50_GGTT_MAP_FW || state == B50_GGTT_VERIFY_FW) {
            if (page >= ggtt_firmware_pages())
                return false;
            ggtt = INTEL_XE2_GGTT_FW_ADDR +
                   page * INTEL_XE2_GGTT_PAGE_BYTES;
            dma = PCIE1_DMA_PCIE_BASE +
                  (PIOS_DMA_PCIE1_GUC_FW_BASE - PIOS_DMA_PCIE1_BASE) +
                  B50_FW_PAYLOAD_OFFSET +
                  (u64)page * INTEL_XE2_GGTT_PAGE_BYTES;
        } else if (state == B50_GGTT_MAP_ADS ||
                   state == B50_GGTT_VERIFY_ADS) {
            u32 ads_bytes;
            if (!intel_guc_min_ads_bytes(guc_artifact.info.private_data_bytes,
                                         &ads_bytes) ||
                page >= ads_bytes / INTEL_XE2_GGTT_PAGE_BYTES)
                return false;
            ggtt = INTEL_XE2_GGTT_ADS_ADDR +
                   page * INTEL_XE2_GGTT_PAGE_BYTES;
            dma = PCIE1_DMA_PCIE_BASE +
                  (PIOS_DMA_PCIE1_GUC_ADS_BASE - PIOS_DMA_PCIE1_BASE) +
                  (u64)page * INTEL_XE2_GGTT_PAGE_BYTES;
        } else {
            if (page >= PIOS_DMA_PCIE1_GUC_Q_SIZE /
                        INTEL_XE2_GGTT_PAGE_BYTES)
                return false;
            ggtt = INTEL_XE2_GGTT_QUEUE_ADDR +
                   page * INTEL_XE2_GGTT_PAGE_BYTES;
            dma = PCIE1_DMA_PCIE_BASE +
                  (PIOS_DMA_PCIE1_GUC_Q_BASE - PIOS_DMA_PCIE1_BASE) +
                  (u64)page * INTEL_XE2_GGTT_PAGE_BYTES;
        }
        return intel_xe2_ggtt_pte_mmio_offset(ggtt, offset_out) &&
               intel_xe2_ggtt_encode_system_pte(
                   dma, INTEL_XE2_GGTT_SYSTEM_PAT_INDEX, pte_out);
    }

static u32 ggtt_step(void *ctx, u64 deadline_ms)
    {
        u32 limit;
        bool verify;
        (void)ctx;
        if (!native.attached ||
            native.firmware_state != B50_FW_STATE_STAGED ||
            native.forcewake_state != B50_FWK_DONE) {
            ggtt_diag.ggtt_detail = B50_GGTT_DIAG_PRECONDITION;
            return ADRV_STEP_FAILED;
        }
        verify = native.ggtt_state == B50_GGTT_VERIFY_FW ||
                 native.ggtt_state == B50_GGTT_VERIFY_ADS ||
                 native.ggtt_state == B50_GGTT_VERIFY_QUEUE;
        if (native.ggtt_state == B50_GGTT_MAP_FW ||
            native.ggtt_state == B50_GGTT_VERIFY_FW) {
            limit = ggtt_firmware_pages();
        } else if (native.ggtt_state == B50_GGTT_MAP_ADS ||
                   native.ggtt_state == B50_GGTT_VERIFY_ADS) {
            if (!intel_guc_min_ads_bytes(guc_artifact.info.private_data_bytes,
                                         &limit)) {
                ggtt_diag.ggtt_detail = B50_GGTT_DIAG_PLAN;
                return ADRV_STEP_FAILED;
            }
            limit /= INTEL_XE2_GGTT_PAGE_BYTES;
        } else {
            limit = PIOS_DMA_PCIE1_GUC_Q_SIZE / INTEL_XE2_GGTT_PAGE_BYTES;
        }
        for (u32 count = 0U;
             count < B50_GGTT_PTES_PER_STEP && native.ggtt_page < limit;
             count++, native.ggtt_page++) {
            u32 offset;
            u64 pte;
            ggtt_diag.ggtt_state = native.ggtt_state;
            ggtt_diag.ggtt_page = native.ggtt_page;
            if (!ggtt_pte(native.ggtt_state, native.ggtt_page, &offset, &pte)) {
                ggtt_diag.ggtt_detail = B50_GGTT_DIAG_PLAN;
                return ADRV_STEP_FAILED;
            }
            ggtt_diag.ggtt_offset = offset;
            ggtt_diag.ggtt_expected = pte;
            if (verify) {
                u64 actual = mmio_read64(PIOS_PCIE1_CPU_WIN_BASE + offset);
                ggtt_diag.ggtt_actual = actual;
                if (actual != pte) {
                    ggtt_diag.ggtt_detail = B50_GGTT_DIAG_READBACK;
                    return ADRV_STEP_FAILED;
                }
            } else {
                mmio_write64(PIOS_PCIE1_CPU_WIN_BASE + offset, pte);
            }
        }
        dsb();
        if (timer_monotonic_ms() > deadline_ms) {
            ggtt_diag.ggtt_detail = B50_GGTT_DIAG_STEP_LATE;
            return ADRV_STEP_FAILED;
        }
        if (native.ggtt_page < limit)
            return ADRV_STEP_PROGRESS;
        native.ggtt_page = 0U;
        switch (native.ggtt_state) {
        case B50_GGTT_MAP_FW:
            native.ggtt_state = B50_GGTT_MAP_ADS;
            break;
        case B50_GGTT_MAP_ADS:
            native.ggtt_state = B50_GGTT_MAP_QUEUE;
            break;
        case B50_GGTT_MAP_QUEUE:
            native.ggtt_state = B50_GGTT_VERIFY_FW;
            break;
        case B50_GGTT_VERIFY_FW:
            native.ggtt_state = B50_GGTT_VERIFY_ADS;
            break;
        case B50_GGTT_VERIFY_ADS:
            native.ggtt_state = B50_GGTT_VERIFY_QUEUE;
            break;
        case B50_GGTT_VERIFY_QUEUE:
            native.ggtt_state = B50_GGTT_DONE;
            return ADRV_STEP_DONE;
        default:
            ggtt_diag.ggtt_detail = B50_GGTT_DIAG_STATE;
            return ADRV_STEP_FAILED;
        }

        return ADRV_STEP_PROGRESS;
}

static u32 boot_prepare_continue(void *ctx, u64 deadline_ms, u32 ads_bytes);

static u32 boot_prepare_step(void *ctx, u64 deadline_ms)
{
    u32 ads_bytes;
    (void)ctx;
    ggtt_diag.prep_state = boot_prep.state;
    ggtt_diag.prep_offset = boot_prep.offset;
    if (!native.attached || native.ggtt_state != B50_GGTT_DONE ||
        !intel_guc_min_ads_bytes(guc_artifact.info.private_data_bytes,
                                 &ads_bytes)) {
        ggtt_diag.prep_detail = B50_PREP_DIAG_PRECONDITION;
        return ADRV_STEP_FAILED;
    }
    return boot_prepare_continue(ctx, deadline_ms, ads_bytes);
}

static u8 *ct_cpu_base(void)
    {
    #ifndef B50_NATIVE_HOST_TEST
        return (u8 *)(usize)(PIOS_DMA_PCIE1_GUC_Q_BASE +
                             B50_CT_BASE_OFFSET);
    #else
        return b50_native_test_pointer(PIOS_DMA_PCIE1_GUC_Q_BASE) +
               B50_CT_BASE_OFFSET;
    #endif
    }

static u32 ct_ggtt(u32 queue_offset)
    {
        return INTEL_XE2_GGTT_QUEUE_ADDR + queue_offset;
    }

static void ct_mmio_send(u64 now)
    {
        for (u32 i = 0U; i < ct_boot.request_len; i++)
            mmio_write(b50_mmio_addr(B50_VF_SW_FLAG0 + i * 4U),
                       ct_boot.request[i]);
        (void)mmio_read(b50_mmio_addr(
            B50_VF_SW_FLAG0 + (ct_boot.request_len - 1U) * 4U));
        mmio_write(b50_mmio_addr(B50_GUC_HOST_INTERRUPT), 0U);
        dsb();
        ct_boot.deadline = now + B50_CT_MMIO_TIMEOUT_MS;
    }

static bool ct_config_request(u32 index)
    {
        static const u16 keys[6] = {
            INTEL_GUC_SELF_CFG_H2G_ADDR,
            INTEL_GUC_SELF_CFG_H2G_DESC,
            INTEL_GUC_SELF_CFG_H2G_SIZE,
            INTEL_GUC_SELF_CFG_G2H_ADDR,
            INTEL_GUC_SELF_CFG_G2H_DESC,
            INTEL_GUC_SELF_CFG_G2H_SIZE
        };
        static const u16 lengths[6] = {2U, 2U, 1U, 2U, 2U, 1U};
        const u64 values[6] = {
            ct_ggtt(B50_CT_H2G_RING_OFFSET),
            ct_ggtt(B50_CT_H2G_DESC_OFFSET),
            INTEL_GUC_CTB_H2G_RING_BYTES,
            ct_ggtt(B50_CT_G2H_RING_OFFSET),
            ct_ggtt(B50_CT_G2H_DESC_OFFSET),
            INTEL_GUC_CTB_G2H_RING_BYTES
        };
        if (index >= 6U ||
            !intel_guc_self_cfg_request(keys[index], lengths[index],
                                        values[index], ct_boot.request))
            return false;
        ct_boot.request_len = 4U;
        return true;
    }

static u32 ct_step(void *ctx, u64 deadline_ms)
    {
        u8 *base = ct_cpu_base();
        u64 now = timer_monotonic_ms();
        struct intel_guc_ct_desc *h2g =
            (struct intel_guc_ct_desc *)(void *)
            (base + (B50_CT_H2G_DESC_OFFSET - B50_CT_BASE_OFFSET));
        struct intel_guc_ct_desc *g2h =
            (struct intel_guc_ct_desc *)(void *)
            (base + (B50_CT_G2H_DESC_OFFSET - B50_CT_BASE_OFFSET));
        u32 *h2g_ring = (u32 *)(void *)
            (base + (B50_CT_H2G_RING_OFFSET - B50_CT_BASE_OFFSET));
        u32 *g2h_ring = (u32 *)(void *)
            (base + (B50_CT_G2H_RING_OFFSET - B50_CT_BASE_OFFSET));
        u32 *hwconfig = (u32 *)(void *)
            (base + (B50_CT_HWCONFIG_OFFSET - B50_CT_BASE_OFFSET));
        (void)ctx;
        if (!native.attached || !guc_boot.ready ||
            now > deadline_ms ||
            B50_CT_BASE_OFFSET + B50_CT_TOTAL_BYTES >
                PIOS_DMA_PCIE1_GUC_Q_SIZE)
            return ADRV_STEP_FAILED;
        switch (ct_boot.state) {
        case B50_CT_ZERO: {
            u32 remain = B50_CT_TOTAL_BYTES - ct_boot.offset;
            u32 chunk = remain < B50_CT_ZERO_CHUNK ?
                remain : B50_CT_ZERO_CHUNK;
            memset(base + ct_boot.offset, 0, chunk);
            dsb();
            ct_boot.offset += chunk;
            if (ct_boot.offset < B50_CT_TOTAL_BYTES)
                return ADRV_STEP_PROGRESS;
            if (!intel_guc_ct_desc_init(h2g) ||
                !intel_guc_ct_desc_init(g2h))
                return ADRV_STEP_FAILED;
            ct_boot.index = 0U;
            ct_boot.state = B50_CT_CFG_SEND;
            return ADRV_STEP_PROGRESS;
        }

        case B50_CT_CFG_SEND:
            if (!ct_config_request(ct_boot.index))
                return ADRV_STEP_FAILED;
            ct_boot.retries = 0U;
            ct_mmio_send(now);
            ct_boot.state = B50_CT_CFG_WAIT;
            return ADRV_STEP_PROGRESS;
        case B50_CT_CFG_WAIT:
        case B50_CT_CONTROL_WAIT: {
            u32 header = mmio_read(b50_mmio_addr(B50_VF_SW_FLAG0));
            int classification = intel_guc_mmio_response_classify(header);
            ct_boot.response = header;
            if (classification == 0 || classification == 2) {
                if (now >= ct_boot.deadline)
                    return ADRV_STEP_FAILED;
                return ADRV_STEP_IDLE;
            }
            if (classification == 3) {
                if (ct_boot.retries++ >= B50_CT_MAX_RETRIES)
                    return ADRV_STEP_FAILED;
                ct_mmio_send(now);
                return ADRV_STEP_PROGRESS;
            }
            if (classification != 1)
                return ADRV_STEP_FAILED;
            if (ct_boot.state == B50_CT_CFG_WAIT) {
                if (++ct_boot.index < 6U)
                    ct_boot.state = B50_CT_CFG_SEND;
                else
                    ct_boot.state = B50_CT_CONTROL_SEND;
            } else {
                ct_boot.state = B50_CT_PROBE_SEND;
            }
            return ADRV_STEP_PROGRESS;
        }
        case B50_CT_CONTROL_SEND:
            intel_guc_control_ctb_request(true, ct_boot.request);
            ct_boot.request_len = 2U;
            ct_boot.retries = 0U;
            ct_mmio_send(now);
            ct_boot.state = B50_CT_CONTROL_WAIT;
            return ADRV_STEP_PROGRESS;
        case B50_CT_PROBE_SEND: {
            u32 payload[3] = {
                ct_ggtt(B50_CT_HWCONFIG_OFFSET), 0U, 4096U
            };
            ct_boot.fence = 1U;
            if (!intel_guc_ct_h2g_push(
                    h2g, h2g_ring, INTEL_GUC_CTB_H2G_RING_DWORDS,
                    (u16)ct_boot.fence, B50_GUC_GET_HWCONFIG,
                    payload, 3U))
                return ADRV_STEP_FAILED;
            mmio_write(b50_mmio_addr(B50_GUC_HOST_INTERRUPT), 0U);
            dsb();
            ct_boot.deadline = now + B50_CT_PROBE_TIMEOUT_MS;
            ct_boot.state = B50_CT_PROBE_WAIT;
            return ADRV_STEP_PROGRESS;
        }
        case B50_CT_PROBE_WAIT: {
            u32 response[16];
            u32 response_dwords = 0U;
            u16 fence = 0U;
            if (g2h->status || h2g->status)
                return ADRV_STEP_FAILED;
            if (!intel_guc_ct_g2h_pop(
                    g2h, g2h_ring, INTEL_GUC_CTB_G2H_RING_DWORDS,
                    response, 16U, &response_dwords, &fence)) {
                if (now >= ct_boot.deadline)
                    return ADRV_STEP_FAILED;
                return ADRV_STEP_IDLE;
            }
            ct_boot.response = response[0];
            if (fence != ct_boot.fence || response_dwords != 1U ||
                intel_guc_mmio_response_classify(response[0]) != -1 ||
                (response[0] & 0x0FFFFFFFU) != 0x30U ||
                hwconfig[0] != 0U)
                return ADRV_STEP_FAILED;
            ct_boot.state = B50_CT_DONE;
            return ADRV_STEP_DONE;
        }
        default:
            return ADRV_STEP_FAILED;
        }
}

static u32 context_step(void *ctx, u64 deadline_ms)
{
    u8 *queue = ct_cpu_base() - B50_CT_BASE_OFFSET;
    u8 *ct_base = queue + B50_CT_BASE_OFFSET;
    struct intel_guc_ct_desc *h2g =
        (struct intel_guc_ct_desc *)(void *)
        (ct_base + (B50_CT_H2G_DESC_OFFSET - B50_CT_BASE_OFFSET));
    struct intel_guc_ct_desc *g2h =
        (struct intel_guc_ct_desc *)(void *)
        (ct_base + (B50_CT_G2H_DESC_OFFSET - B50_CT_BASE_OFFSET));
    u32 *h2g_ring = (u32 *)(void *)
        (ct_base + (B50_CT_H2G_RING_OFFSET - B50_CT_BASE_OFFSET));
    u32 *g2h_ring = (u32 *)(void *)
        (ct_base + (B50_CT_G2H_RING_OFFSET - B50_CT_BASE_OFFSET));
    u64 now = timer_monotonic_ms();
    (void)ctx;
    if (!native.attached || !guc_boot.ready ||
        ct_boot.state != B50_CT_DONE || now > deadline_ms ||
        B50_CONTEXT_LRC_OFFSET + INTEL_BMG_LRC_TOTAL_BYTES >
            PIOS_DMA_PCIE1_GUC_Q_SIZE)
        return ADRV_STEP_FAILED;
    switch (context_boot.state) {
    case B50_CONTEXT_BUILD: {
        u64 descriptor;
        u32 action[12];
        u32 lrc_ggtt = ct_ggtt(B50_CONTEXT_LRC_OFFSET);
        u32 result_ggtt = ct_ggtt(B50_CONTEXT_RESULT_OFFSET);
        u32 completion_ggtt =
            ct_ggtt(B50_CONTEXT_COMPLETION_OFFSET);
        if (!intel_bmg_lrc_build(
                queue + B50_CONTEXT_LRC_OFFSET,
                PIOS_DMA_PCIE1_GUC_Q_SIZE - B50_CONTEXT_LRC_OFFSET,
                lrc_ggtt, result_ggtt, completion_ggtt,
                &context_boot.ring_tail) ||
            !intel_bmg_lrc_descriptor(lrc_ggtt, &descriptor) ||
            !intel_bmg_register_context_action(
                B50_CONTEXT_GUC_ID, descriptor, action))
            return ADRV_STEP_FAILED;
        *(u32 *)(void *)(queue + B50_CONTEXT_RESULT_OFFSET) = 0U;
        *(u32 *)(void *)(queue + B50_CONTEXT_COMPLETION_OFFSET) = 0U;
        context_boot.descriptor_lo = (u32)descriptor;
        context_boot.descriptor_hi = (u32)(descriptor >> 32);
        context_boot.fence = 2U;
        if (!intel_guc_ct_h2g_push(
                h2g, h2g_ring, INTEL_GUC_CTB_H2G_RING_DWORDS,
                (u16)context_boot.fence, (u16)action[0],
                &action[1], 11U))
            return ADRV_STEP_FAILED;
        mmio_write(b50_mmio_addr(B50_GUC_HOST_INTERRUPT), 0U);
        dsb();
        context_boot.deadline = now + B50_CONTEXT_TIMEOUT_MS;
        context_boot.state = B50_CONTEXT_WAIT;
        return ADRV_STEP_PROGRESS;
    }

    case B50_CONTEXT_WAIT: {
        u32 response[16];
        u32 response_dwords = 0U;
        u16 fence = 0U;
        if (h2g->status || g2h->status)
            return ADRV_STEP_FAILED;
        if (!intel_guc_ct_g2h_pop(
                g2h, g2h_ring, INTEL_GUC_CTB_G2H_RING_DWORDS,
                response, 16U, &response_dwords, &fence)) {
            if (now >= context_boot.deadline)
                return ADRV_STEP_FAILED;
            return ADRV_STEP_IDLE;
        }
        context_boot.response = response[0];
        if (fence != context_boot.fence || response_dwords == 0U ||
            intel_guc_mmio_response_classify(response[0]) != 1)
            return ADRV_STEP_FAILED;
        context_boot.state = B50_CONTEXT_DONE;
        return ADRV_STEP_DONE;
    }
    default:
        return ADRV_STEP_FAILED;
    }
}

static u32 submit_step(void *ctx, u64 deadline_ms)
{
    u8 *queue = ct_cpu_base() - B50_CT_BASE_OFFSET;
    u8 *ct_base = queue + B50_CT_BASE_OFFSET;
    struct intel_guc_ct_desc *h2g =
        (struct intel_guc_ct_desc *)(void *)
        (ct_base + (B50_CT_H2G_DESC_OFFSET - B50_CT_BASE_OFFSET));
    struct intel_guc_ct_desc *g2h =
        (struct intel_guc_ct_desc *)(void *)
        (ct_base + (B50_CT_G2H_DESC_OFFSET - B50_CT_BASE_OFFSET));
    u32 *h2g_ring = (u32 *)(void *)
        (ct_base + (B50_CT_H2G_RING_OFFSET - B50_CT_BASE_OFFSET));
    u32 *g2h_ring = (u32 *)(void *)
        (ct_base + (B50_CT_G2H_RING_OFFSET - B50_CT_BASE_OFFSET));
    volatile u32 *result = (volatile u32 *)(void *)
        (queue + B50_CONTEXT_RESULT_OFFSET);
    volatile u32 *completion = (volatile u32 *)(void *)
        (queue + B50_CONTEXT_COMPLETION_OFFSET);
    u64 now = timer_monotonic_ms();
    (void)ctx;
    if (!native.attached || !guc_boot.ready ||
        context_boot.state != B50_CONTEXT_DONE || now > deadline_ms)
        return ADRV_STEP_FAILED;
    switch (submit_boot.state) {
    case B50_SUBMIT_ENABLE_SEND: {
        u32 action[3];
        intel_bmg_sched_enable_action(B50_CONTEXT_GUC_ID, action);
        submit_boot.fence = 3U;
        if (!intel_guc_ct_h2g_push(
                h2g, h2g_ring, INTEL_GUC_CTB_H2G_RING_DWORDS,
                (u16)submit_boot.fence, (u16)action[0],
                &action[1], 2U))
            return ADRV_STEP_FAILED;
        dmb();
        mmio_write(b50_mmio_addr(B50_GUC_HOST_INTERRUPT), 0U);
        dsb();
        submit_boot.deadline = now + B50_SUBMIT_TIMEOUT_MS;
        submit_boot.state = B50_SUBMIT_ENABLE_WAIT;
        return ADRV_STEP_PROGRESS;
    }
    case B50_SUBMIT_ENABLE_WAIT: {
        u32 response[16];
        u32 response_dwords = 0U;
        u16 fence = 0U;
        if (h2g->status || g2h->status)
            return ADRV_STEP_FAILED;
        if (g2h->head != g2h->tail &&
            intel_guc_ct_g2h_pop(
                g2h, g2h_ring, INTEL_GUC_CTB_G2H_RING_DWORDS,
                response, 16U, &response_dwords, &fence)) {
            submit_boot.response = response[0];
            if (response_dwords == 3U &&
                response[0] == B50_GUC_SCHED_MODE_DONE &&
                response[1] == B50_CONTEXT_GUC_ID &&
                response[2] == 1U)
                submit_boot.mode_done = 1U;
        }
        if (submit_boot.mode_done) {
            submit_boot.state = B50_SUBMIT_SCHED_SEND;
            return ADRV_STEP_PROGRESS;
        }
        if (now >= submit_boot.deadline)
            return ADRV_STEP_FAILED;
        return ADRV_STEP_IDLE;
    }
    case B50_SUBMIT_SCHED_SEND: {
        u32 action[2];
        intel_bmg_sched_context_action(B50_CONTEXT_GUC_ID, action);
        submit_boot.fence = 4U;
        if (!intel_guc_ct_h2g_push(
                h2g, h2g_ring, INTEL_GUC_CTB_H2G_RING_DWORDS,
                (u16)submit_boot.fence, (u16)action[0],
                &action[1], 1U))
            return ADRV_STEP_FAILED;
        dmb();
        mmio_write(b50_mmio_addr(B50_GUC_HOST_INTERRUPT), 0U);
        dsb();
        submit_boot.deadline = now + B50_SUBMIT_TIMEOUT_MS;
        submit_boot.state = B50_SUBMIT_WAIT;
        return ADRV_STEP_PROGRESS;
    }
    case B50_SUBMIT_WAIT: {
        u32 response[16];
        u32 response_dwords = 0U;
        u16 fence = 0U;
        if (h2g->status || g2h->status)
            return ADRV_STEP_FAILED;
        if (g2h->head != g2h->tail &&
            intel_guc_ct_g2h_pop(
                g2h, g2h_ring, INTEL_GUC_CTB_G2H_RING_DWORDS,
                response, 16U, &response_dwords, &fence))
            submit_boot.response = response[0];
        dmb();
        submit_boot.result = *result;
        submit_boot.completion = *completion;
        if (submit_boot.completion == INTEL_BMG_STORE_COMPLETION_VALUE) {
            if (submit_boot.result != INTEL_BMG_STORE_RESULT_VALUE)
                return ADRV_STEP_FAILED;
            submit_boot.state = B50_SUBMIT_DONE;
            return ADRV_STEP_DONE;
        }
        if (submit_boot.completion != 0U ||
            now >= submit_boot.deadline) {
            submit_engine_diag[0] =
                mmio_read(b50_mmio_addr(B50_CCS0_RING_HEAD));
            submit_engine_diag[1] =
                mmio_read(b50_mmio_addr(B50_CCS0_RING_TAIL));
            submit_engine_diag[2] =
                mmio_read(b50_mmio_addr(B50_CCS0_RING_START));
            submit_engine_diag[3] =
                mmio_read(b50_mmio_addr(B50_CCS0_RING_CTL));
            submit_engine_diag[4] =
                mmio_read(b50_mmio_addr(B50_CCS0_HWS_PGA));
            submit_engine_diag[5] =
                mmio_read(b50_mmio_addr(B50_CCS0_CONTEXT_CONTROL));
            submit_engine_diag[6] =
                mmio_read(b50_mmio_addr(B50_CCS0_ACTHD));
            submit_engine_diag[7] =
                mmio_read(b50_mmio_addr(B50_CCS0_IPEHR));
            return ADRV_STEP_FAILED;
        }
        return ADRV_STEP_IDLE;
    }
    default:
        return ADRV_STEP_FAILED;
    }
}

static u32 boot_prepare_continue(void *ctx, u64 deadline_ms, u32 ads_bytes)
{
    u8 *ads;
    u8 *log;
    u32 limit, chunk;
    (void)ctx;
#ifndef B50_NATIVE_HOST_TEST
    ads = (u8 *)(usize)PIOS_DMA_PCIE1_GUC_ADS_BASE;
    log = (u8 *)(usize)PIOS_DMA_PCIE1_GUC_Q_BASE;
#else
    ads = b50_native_test_pointer(PIOS_DMA_PCIE1_GUC_ADS_BASE);
    log = b50_native_test_pointer(PIOS_DMA_PCIE1_GUC_Q_BASE);
#endif
    if (boot_prep.state == B50_PREP_ZERO_ADS ||
        boot_prep.state == B50_PREP_ZERO_LOG) {
        limit = boot_prep.state == B50_PREP_ZERO_ADS ?
            ads_bytes + B50_PREP_GUARD_BYTES :
            INTEL_GUC_LOG_BYTES + B50_PREP_GUARD_BYTES;
        chunk = limit - boot_prep.offset;
        if (chunk > B50_PREP_CHUNK_BYTES)
            chunk = B50_PREP_CHUNK_BYTES;
        memset((boot_prep.state == B50_PREP_ZERO_ADS ? ads : log) +
               boot_prep.offset, 0, chunk);
        dsb();
        boot_prep.offset += chunk;
        if (boot_prep.offset == limit) {
            boot_prep.offset = 0U;
            boot_prep.state = boot_prep.state == B50_PREP_ZERO_ADS ?
                B50_PREP_ZERO_LOG : B50_PREP_BUILD;
        }
        if (timer_monotonic_ms() > deadline_ms) {
            ggtt_diag.prep_detail = B50_PREP_DIAG_STEP_LATE;
            return ADRV_STEP_FAILED;
        }
        return ADRV_STEP_PROGRESS;
    }
    if (boot_prep.state == B50_PREP_BUILD) {
        if (!intel_guc_min_boot_build(
                ads, PIOS_DMA_PCIE1_GUC_ADS_SIZE,
                log, PIOS_DMA_PCIE1_GUC_Q_SIZE,
                INTEL_XE2_GGTT_ADS_ADDR, INTEL_XE2_GGTT_QUEUE_ADDR,
                guc_artifact.info.private_data_bytes, 0xE212U,
                (u8)boot_prep.revision, &boot_layout)) {
            ggtt_diag.prep_detail = B50_PREP_DIAG_BUILD;
            return ADRV_STEP_FAILED;
        }
        for (u32 i = 0U; i < B50_PREP_GUARD_BYTES; i++) {
            ads[boot_layout.ads_bytes + i] = 0xA5U;
            log[boot_layout.log_bytes + i] = 0x5AU;
        }
        dsb();
        boot_prep.state = B50_PREP_DONE;
        return ADRV_STEP_DONE;
    }
    ggtt_diag.prep_detail = B50_PREP_DIAG_STATE;
    return ADRV_STEP_FAILED;
}

static void native_bottom(const struct airq_record *record, void *ctx)
{
    (void)ctx;
    if (!record || core_id() || record->origin_core || record->target_core)
        return;
    u64 sequence = trigger.sequence;
    dmb();
    if (sequence == continuation.acknowledged)
        return;
    uart_puts(quarantine());
    continuation.acknowledged = sequence;
}

static const u32 buses[] = {1U, 2U, 3U, 4U, 5U};
static const u32 devices[] = {0U, 8U, 0U, 1U, 0U};
static const u32 identities[] = {
    0x874810B5U, 0x874810B5U, 0xE2FF8086U, 0xE2F08086U,
    0xE2128086U
};
static const u32 aer_offsets[] = {
    0xFB4U, 0xFB4U, 0x100U, 0x100U, 0U
};
#define B50_PATH_KNOWN_CORR_MASK 0x000020C1U

static bool path_aer_clean(u32 index)
{
    u32 offset = aer_offsets[index];
    u32 header;
    u32 uncorr;
    u32 corr;
    if (!offset)
        return true;
    header = pcie1_cfg_read(buses[index], devices[index], 0U, offset);
    uncorr = pcie1_cfg_read(buses[index], devices[index], 0U, offset + 4U);
    corr = pcie1_cfg_read(buses[index], devices[index], 0U, offset + 16U);
    if ((header & 0xFFFFU) != 1U || uncorr ||
        (corr & ~B50_PATH_KNOWN_CORR_MASK))
        return false;
    if (corr) {
        pcie1_cfg_write(buses[index], devices[index], 0U,
                        offset + 16U, corr);
        dsb();
    }
    return pcie1_cfg_read(buses[index], devices[index], 0U, offset + 4U) == 0U &&
           pcie1_cfg_read(buses[index], devices[index], 0U,
                          offset + 16U) == 0U;
}

static bool revoke_bme(void)
{
    bool ok = true;
    for (u32 i = 0; i < 5U; i++) {
        if (pcie1_cfg_read(buses[i], devices[i], 0U, 0U) != identities[i]) {
            ok = false;
            continue;
        }
        u32 command = pcie1_cfg_read(buses[i], devices[i], 0U, 4U);
        if (command == ~0U) {
            ok = false;
            continue;
        }
        /* Never echo PCI status W1C bits in the upper halfword. */
        pcie1_cfg_write(buses[i], devices[i], 0U, 4U,
                        (command & 0xFFFFU) & ~4U);
        if (pcie1_cfg_read(buses[i], devices[i], 0U, 4U) & 4U)
            ok = false;
    }
    dsb();
    return ok;
}

static bool enable_bme(void)
{
    for (u32 i = 0U; i < 5U; i++) {
        u32 command;
        if (pcie1_cfg_read(buses[i], devices[i], 0U, 0U) != identities[i])
            goto fail;
        command = pcie1_cfg_read(buses[i], devices[i], 0U, 4U);
        if (command == 0xFFFFFFFFU || (command & 3U) != 2U)
            goto fail;
        pcie1_cfg_write(buses[i], devices[i], 0U, 4U,
                        (command & 0xFFFFU) | 4U);
        if ((pcie1_cfg_read(buses[i], devices[i], 0U, 4U) & 7U) != 6U)
            goto fail;
    }
    dsb();
    return true;
fail:
    (void)revoke_bme();
    return false;
}

static bool guc_guards_intact(void)
{
    u8 *fw;
    u8 *ads;
    u8 *log;
#ifndef B50_NATIVE_HOST_TEST
    fw = (u8 *)(usize)PIOS_DMA_PCIE1_GUC_FW_BASE;
    ads = (u8 *)(usize)PIOS_DMA_PCIE1_GUC_ADS_BASE;
    log = (u8 *)(usize)PIOS_DMA_PCIE1_GUC_Q_BASE;
#else
    fw = b50_native_test_pointer(PIOS_DMA_PCIE1_GUC_FW_BASE);
    ads = b50_native_test_pointer(PIOS_DMA_PCIE1_GUC_ADS_BASE);
    log = b50_native_test_pointer(PIOS_DMA_PCIE1_GUC_Q_BASE);
#endif
    if (boot_layout.ads_bytes == 0U || boot_layout.log_bytes == 0U)
        return false;
    for (u32 i = 0U; i < B50_FW_GUARD_BYTES; i++)
        if (fw[i] != 0xA5U ||
            fw[PIOS_DMA_PCIE1_GUC_FW_SIZE -
               B50_FW_GUARD_BYTES + i] != 0x5AU)
            return false;
    for (u32 i = 0U; i < B50_PREP_GUARD_BYTES; i++)
        if (ads[boot_layout.ads_bytes + i] != 0xA5U ||
            log[boot_layout.log_bytes + i] != 0x5AU)
            return false;
    return true;
}

static u32 guc_boot_step(void *ctx, u64 deadline_ms)
{
    struct intel_xe2_wopcm_plan wopcm;
    struct pcie1_aer_snapshot aer;
    u64 now = timer_monotonic_ms();
    u32 offset, value, actual_size, actual_base;
    (void)ctx;
    if (!native.attached || boot_prep.state != B50_PREP_DONE ||
        native.ggtt_state != B50_GGTT_DONE ||
        !intel_bmg_wopcm_plan(
            guc_artifact.info.css_bytes + guc_artifact.info.ucode_bytes,
            &wopcm) ||
        now > deadline_ms)
        return ADRV_STEP_FAILED;
    boot_diag.state = guc_boot.state;
    boot_diag.wopcm_size_expected = wopcm.size_readback;
    boot_diag.wopcm_offset_expected = wopcm.offset_readback;
    switch (guc_boot.state) {
    case B50_BOOT_ACQUIRE_GT:
        mmio_write(b50_mmio_addr(B50_FORCEWAKE_GT), B50_FORCEWAKE_REQUEST);
        dsb();
        guc_boot.deadline = now + B50_FORCEWAKE_TIMEOUT_MS;
        guc_boot.state = B50_BOOT_WAIT_GT;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_WAIT_GT:
        value = mmio_read(b50_mmio_addr(B50_FORCEWAKE_ACK_GT));
        if (value == 0xFFFFFFFFU || now >= guc_boot.deadline)
            return ADRV_STEP_FAILED;
        if (!(value & B50_FORCEWAKE_ACK))
            return ADRV_STEP_IDLE;
        guc_boot.state = B50_BOOT_WOPCM;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_RESET:
        mmio_write(b50_mmio_addr(B50_GDRST), B50_GUC_RESET_BIT);
        dsb();
        guc_boot.deadline = now + 5U;
        guc_boot.state = B50_BOOT_WAIT_RESET;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_WAIT_RESET:
        value = mmio_read(b50_mmio_addr(B50_GDRST));
        if (value == 0xFFFFFFFFU || now >= guc_boot.deadline)
            return ADRV_STEP_FAILED;
        if (value & B50_GUC_RESET_BIT)
            return ADRV_STEP_IDLE;
        value = mmio_read(b50_mmio_addr(B50_GUC_STATUS));
        if (!(value & B50_GUC_MIA_RESET))
            return ADRV_STEP_FAILED;
        guc_boot.index = 0U;
        guc_boot.state = B50_BOOT_PAT;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_PAT:
        for (u32 n = 0U; n < 8U && guc_boot.index < 28U;
             n++, guc_boot.index++) {
            if (!intel_bmg_pat_entry(guc_boot.index, &offset, &value))
                return ADRV_STEP_FAILED;
            mmio_write(b50_mmio_addr(offset), value);
        }
        dsb();
        if (guc_boot.index < 28U)
            return ADRV_STEP_PROGRESS;
        if (!intel_bmg_pat_entry(2U, &offset, &value) ||
            mmio_read(b50_mmio_addr(offset)) != value)
            return ADRV_STEP_FAILED;
        guc_boot.index = 0U;
        guc_boot.state = B50_BOOT_MOCS;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_MOCS:
        for (u32 n = 0U; n < 8U && guc_boot.index < 16U;
             n++, guc_boot.index++) {
            if (!intel_bmg_mocs_entry(guc_boot.index, &offset, &value))
                return ADRV_STEP_FAILED;
            mmio_write(b50_mmio_addr(offset), value);
        }
        dsb();
        if (guc_boot.index < 16U)
            return ADRV_STEP_PROGRESS;
        if (!intel_bmg_mocs_entry(3U, &offset, &value) ||
            mmio_read(b50_mmio_addr(offset)) != value)
            return ADRV_STEP_FAILED;
        guc_boot.state = B50_BOOT_TLB_INV;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_TLB_INV:
        mmio_write(b50_mmio_addr(B50_GUC_TLB_INV_DESC1), 1U << 6);
        mmio_write(b50_mmio_addr(B50_GUC_TLB_INV_DESC0), 1U);
        dsb();
        guc_boot.deadline = now + 50U;
        guc_boot.state = B50_BOOT_TLB_WAIT;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_TLB_WAIT:
        value = mmio_read(b50_mmio_addr(B50_GUC_TLB_INV_DESC0));
        if (value == 0xFFFFFFFFU || now >= guc_boot.deadline)
            return ADRV_STEP_FAILED;
        if (value & 1U)
            return ADRV_STEP_IDLE;
        guc_boot.state = B50_BOOT_PARAMS;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_WOPCM:
        actual_size = 0U;
        actual_base = 0U;
        value = mmio_read(b50_mmio_addr(B50_GUC_WOPCM_SIZE));
        boot_diag.wopcm_size_raw = value;
        if (value & 1U) {
            if (!intel_bmg_wopcm_locked_size_valid(
                    value,
                    guc_artifact.info.css_bytes +
                    guc_artifact.info.ucode_bytes,
                    wopcm.guc_bytes, &actual_size))
                return ADRV_STEP_FAILED;
        } else {
            mmio_write(b50_mmio_addr(B50_GUC_WOPCM_SIZE),
                       wopcm.size_write);
            boot_diag.wopcm_size_raw =
                mmio_read(b50_mmio_addr(B50_GUC_WOPCM_SIZE));
            if (!intel_bmg_wopcm_locked_size_valid(
                    boot_diag.wopcm_size_raw,
                    guc_artifact.info.css_bytes +
                    guc_artifact.info.ucode_bytes,
                    wopcm.guc_bytes, &actual_size))
                return ADRV_STEP_FAILED;
        }
        boot_diag.wopcm_size_expected = actual_size | 1U;
        value = mmio_read(b50_mmio_addr(B50_GUC_WOPCM_OFFSET));
        boot_diag.wopcm_offset_raw = value;
        if (value & 1U) {
            if (!intel_bmg_wopcm_locked_offset_valid(
                    value, actual_size, wopcm.total_bytes, &actual_base))
                return ADRV_STEP_FAILED;
        } else {
            mmio_write(b50_mmio_addr(B50_GUC_WOPCM_OFFSET),
                       wopcm.offset_write);
            boot_diag.wopcm_offset_raw =
                mmio_read(b50_mmio_addr(B50_GUC_WOPCM_OFFSET));
            if (!intel_bmg_wopcm_locked_offset_valid(
                    boot_diag.wopcm_offset_raw, actual_size,
                    wopcm.total_bytes, &actual_base))
                return ADRV_STEP_FAILED;
        }
        boot_diag.wopcm_offset_expected =
            actual_base |
            (boot_diag.wopcm_offset_raw & 3U);
        guc_boot.state = B50_BOOT_RESET;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_PARAMS:
        if (!guc_guards_intact())
            return ADRV_STEP_FAILED;
        mmio_write(b50_mmio_addr(B50_GUC_SOFT_SCRATCH), 0U);
        for (u32 i = 0U; i < INTEL_GUC_CTL_DWORDS; i++)
            mmio_write(b50_mmio_addr(B50_GUC_SOFT_SCRATCH +
                                     (i + 1U) * 4U),
                       boot_layout.params[i]);
        mmio_write(b50_mmio_addr(B50_GUC_SHIM_CONTROL),
                   intel_bmg_guc_shim_control());
        if (mmio_read(b50_mmio_addr(B50_GUC_SHIM_CONTROL)) !=
            intel_bmg_guc_shim_control())
            return ADRV_STEP_FAILED;
        mmio_write(b50_mmio_addr(B50_GT_PM_CONFIG), 1U);
        value = mmio_read(b50_mmio_addr(B50_PMINTRMSK));
        mmio_write(b50_mmio_addr(B50_PMINTRMSK), value & ~(1U << 9));
        mmio_write(b50_mmio_addr(B50_GUC_RSA_SCRATCH0),
                   INTEL_XE2_GGTT_FW_ADDR + guc_artifact.info.rsa_offset);
        dsb();
        pcie1_aer_snapshot(&aer, false);
        if (aer.uncorr || aer.corr)
            return ADRV_STEP_FAILED;
        guc_boot.state = B50_BOOT_BME;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_BME:
        if (!enable_bme())
            return ADRV_STEP_FAILED;
        guc_boot.state = B50_BOOT_DMA_START;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_DMA_START:
        mmio_write(b50_mmio_addr(B50_GUC_DMA_ADDR0_LO),
                   INTEL_XE2_GGTT_FW_ADDR);
        mmio_write(b50_mmio_addr(B50_GUC_DMA_ADDR0_HI), 8U << 16);
        mmio_write(b50_mmio_addr(B50_GUC_DMA_ADDR1_LO), 0x2000U);
        mmio_write(b50_mmio_addr(B50_GUC_DMA_ADDR1_HI), 7U << 16);
        mmio_write(b50_mmio_addr(B50_GUC_DMA_COPY_SIZE),
                   guc_artifact.info.css_bytes +
                   guc_artifact.info.ucode_bytes);
        dsb();
        mmio_write(b50_mmio_addr(B50_GUC_DMA_CTRL),
                   B50_GUC_DMA_START_VALUE);
        dsb();
        guc_boot.deadline = now + 100U;
        guc_boot.state = B50_BOOT_DMA_WAIT;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_DMA_WAIT:
        guc_boot.dma_ctrl = mmio_read(b50_mmio_addr(B50_GUC_DMA_CTRL));
        if (guc_boot.dma_ctrl == 0xFFFFFFFFU || now >= guc_boot.deadline)
            return ADRV_STEP_FAILED;
        if (guc_boot.dma_ctrl & B50_GUC_DMA_START_BIT)
            return ADRV_STEP_IDLE;
        mmio_write(b50_mmio_addr(B50_GUC_DMA_CTRL),
                   B50_GUC_DMA_DISABLE);
        dsb();
        guc_boot.deadline = now + 3000U;
        guc_boot.state = B50_BOOT_UCODE_WAIT;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_UCODE_WAIT:
        guc_boot.last_status = mmio_read(b50_mmio_addr(B50_GUC_STATUS));
        value = (u32)intel_guc_load_status_classify(guc_boot.last_status);
        if ((i32)value < 0 || now >= guc_boot.deadline)
            return ADRV_STEP_FAILED;
        if (value == 0U)
            return ADRV_STEP_IDLE;
        pcie1_aer_snapshot(&aer, false);
        if (aer.uncorr || aer.corr || !guc_guards_intact())
            return ADRV_STEP_FAILED;
        guc_boot.ready = 1U;
        guc_boot.state = B50_BOOT_RELEASE_GT;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_RELEASE_GT:
        mmio_write(b50_mmio_addr(B50_FORCEWAKE_GT), B50_FORCEWAKE_RELEASE);
        dsb();
        guc_boot.deadline = now + B50_FORCEWAKE_TIMEOUT_MS;
        guc_boot.state = B50_BOOT_WAIT_GT_OFF;
        return ADRV_STEP_PROGRESS;
    case B50_BOOT_WAIT_GT_OFF:
        value = mmio_read(b50_mmio_addr(B50_FORCEWAKE_ACK_GT));
        if (value == 0xFFFFFFFFU || now >= guc_boot.deadline)
            return ADRV_STEP_FAILED;
        if (value & B50_FORCEWAKE_ACK)
            return ADRV_STEP_IDLE;
        guc_boot.state = B50_BOOT_DONE;
        return ADRV_STEP_DONE;
    default:
        return ADRV_STEP_FAILED;
    }
}

static void guc_boot_abort(void)
{
    if (!native.attached)
        return;
    mmio_write(b50_mmio_addr(B50_GDRST), B50_GUC_RESET_BIT);
    mmio_write(b50_mmio_addr(B50_FORCEWAKE_GT), B50_FORCEWAKE_RELEASE);
    dsb();
    guc_boot.ready = 0U;
}

static bool live_valid(void)
{
    u64 deadline = timer_monotonic_ms() + B50_LIVE_VALIDATE_MS;
    struct lzero_status status;
    struct pcie1_status topology;
    struct pcie1_aer_snapshot aer;
    static const u32 offsets[] = {
        0x402CU, 0x4030U, 0x4034U, 0x4038U, 0x403CU, 0x4040U,
        0x4044U, 0x4048U, 0x40ACU, 0x40B0U, 0x40B4U, 0x40B8U,
        0x40BCU, 0x40C0U, 0x400CU, 0x4010U, 0x4070U, 0x4080U,
        0x4084U, 0x20U
    };
    static const u32 values[] = {
        0U, 0U, PCIE1_BAR2_SIZE_ENC, 0x10U, 0U, 0U, 0U, 0U, 0U, 0U,
        (u32)PIOS_DMA_PCIE1_BASE | 1U, 0U, 0U, 0U,
        0x80000000U, 0U, 0x80F08000U, 0x1BU, 0x1BU, 0x80F08000U
    };
    validation_stage = 10U;
    if (!pcie1_link_up())
        return false;
    pcie1_status(&topology);
    validation_stage = 20U;
    if (topology.scan_truncated || topology.malformed_topology ||
        topology.ep_count == 0U || topology.ep_count > PCIE1_SCAN_MAX)
        return false;
    for (u32 i = 0; i < 5U; i++) {
        validation_stage = 30U + i;
        if (pcie1_cfg_read(buses[i], devices[i], 0U, 0U) != identities[i] ||
            (pcie1_cfg_read(buses[i], devices[i], 0U, 4U) & 7U) !=
                (i < 5U ? (guc_boot.ready ? 6U : 2U) : 0U))
            return false;
        if (i < 4U) {
            u32 range = pcie1_cfg_read(buses[i], devices[i], 0U, 0x18U);
            if (((range >> 8) & 255U) > 5U ||
                ((range >> 16) & 255U) < 5U ||
                pcie1_cfg_read(buses[i], devices[i], 0U, 0x20U) != 0x80F08000U)
                return false;
        }
        for (u32 i = 0U; i < topology.ep_count; i++) {
            const struct pcie1_ep *ep = &topology.eps[i];
            if (!pcie1_is_b50(ep->vendor, ep->device) ||
                (ep->bus == 5U && ep->dev == 0U && ep->func == 0U))
                continue;
            if (pcie1_cfg_read(ep->bus, ep->dev, ep->func, 4U) & 4U)
                return false;
        }
        if (!path_aer_clean(i))
            return false;
    }
    validation_stage = 40U;
    if (!pcie1_link_gen3_active(2U, 8U, 0U))
        return false;
    lzero_status(&status);
    validation_stage = 50U;
    if (!status.bars_probed || !status.bar0_mapped ||
        status.gpu_bus != 5U || status.gpu_dev || status.gpu_func ||
        status.bar0_size != 0x1000000ULL ||
        (pcie1_cfg_read(5U, 0U, 0U, 0x10U) & ~8U) != 0x80000004U ||
        pcie1_cfg_read(5U, 0U, 0U, 0x14U) != 0U)
        return false;
    validation_stage = 60U;
    for (u32 i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        u64 addr = PIOS_PCIE1_RC_BASE + offsets[i];
        if (!mmu_device_read32_valid(addr) || mmio_read(addr) != values[i])
            return false;
    }
    validation_stage = 70U;
    if (!mmu_device_read32_valid(PIOS_PCIE1_RC_BASE + 0x4008U) ||
        ((mmio_read(PIOS_PCIE1_RC_BASE + 0x4008U) >> 27) & 31U) !=
            PCIE1_BAR2_SIZE_ENC)
        return false;
    pcie1_aer_snapshot(&aer, false);
    validation_stage = 80U;
    if (!aer.aer_offset || aer.uncorr || aer.corr ||
        !mmu_active_nc_l2_range_valid(PIOS_DMA_PCIE1_BASE,
                                      PIOS_DMA_PCIE1_SIZE))
        return false;
    validation_stage = 90U;
    for (u64 off = 0; off < 0x1000000ULL; off += B50_L2_BLOCK_SIZE)
        if (timer_monotonic_ms() >= deadline ||
            !mmu_device_read32_valid(PIOS_PCIE1_CPU_WIN_BASE + off) ||
            !mmu_device_read32_valid(PIOS_PCIE1_CPU_WIN_BASE + off +
                                     B50_L2_BLOCK_SIZE - sizeof(u32)))
            return false;
    validation_stage = 100U;
    for (u32 i = 0; i < 3U; i++)
        if (mmio_read(PIOS_PCIE1_CPU_WIN_BASE + 0xD8CU) != 0x05004000U)
            return false;
    validation_stage = 110U;
    return timer_monotonic_ms() < deadline;
}

static bool owned_slice_check(void)
{
    struct pcie1_containment_dma_handle handle;
    struct pcie1_containment_dma_span span;
    u64 now = timer_monotonic_ms();
    u64 request = containment.owner.last_request_id + 1U;
    if (!request || !pcie1_containment_dma_acquire(
            &containment, &authority.endpoint, 0U, request, 64U,
            PCIE1_CONTAINMENT_FROM_DEVICE, now, 50U, &handle))
        return false;
    if (!pcie1_containment_dma_span_get(&containment, &handle, 0U, &span) ||
        span.payload_cpu_phys < PIOS_DMA_PCIE1_BASE + 64U ||
        span.capacity != PCIE1_CONTAINMENT_DMA_PAYLOAD_BYTES ||
        span.payload_cpu_phys > PIOS_DMA_PCIE1_BASE +
                               PIOS_DMA_PCIE1_CANARY_SIZE -
                               span.capacity - 64U)
        return false;
    u64 base = span.payload_cpu_phys - 64U;
#ifndef B50_NATIVE_HOST_TEST
    volatile u8 *bytes = (volatile u8 *)(usize)base;
#else
    volatile u8 *bytes = b50_native_test_pointer(base);
#endif
    for (u32 i = 0; i < 64U; i++) {
        bytes[i] = 0xA5U;
        bytes[64U + span.capacity + i] = 0x5AU;
        bytes[64U + i] = (u8)(i ^ handle.generation);
    }
    dsb();
    bool intact = true;
    for (u32 i = 0; i < 64U; i++)
        if (bytes[i] != 0xA5U || bytes[64U + span.capacity + i] != 0x5AU ||
            bytes[64U + i] != (u8)(i ^ handle.generation))
            intact = false;
    /* Only the requested payload and its guards were touched. */
    for (u32 i = 0; i < 64U; i++) {
        bytes[i] = bytes[64U + i] = 0xDDU;
        bytes[64U + span.capacity + i] = 0xDDU;
    }
    dsb();
    return intact &&
        pcie1_containment_dma_cancel(&containment, &authority.endpoint, &handle, 0U) &&
        pcie1_containment_dma_release(&containment, &authority.endpoint, &handle, 0U);
}

static bool probe_bar(struct pcie1_bar_lease_bar *bar)
{
    u32 command = pcie1_cfg_read(5U, 0U, 0U, 4U);
    bar->config_lo = pcie1_cfg_read(5U, 0U, 0U, 0x10U);
    bar->config_hi = pcie1_cfg_read(5U, 0U, 0U, 0x14U);
    if ((command & 7U) != 2U ||
        (bar->config_lo & ~8U) != 0x80000004U ||
        bar->config_hi != 0U)
        return false;
    pcie1_cfg_write(5U, 0U, 0U, 4U, (command & 0xFFFFU) & ~7U);
    if (pcie1_cfg_read(5U, 0U, 0U, 4U) & 7U)
        return false;
    pcie1_cfg_write(5U, 0U, 0U, 0x10U, ~0U);
    pcie1_cfg_write(5U, 0U, 0U, 0x14U, ~0U);
    bar->probe_lo = pcie1_cfg_read(5U, 0U, 0U, 0x10U);
    bar->probe_hi = pcie1_cfg_read(5U, 0U, 0U, 0x14U);
    pcie1_cfg_write(5U, 0U, 0U, 0x14U, bar->config_hi);
    pcie1_cfg_write(5U, 0U, 0U, 0x10U, bar->config_lo);
    dsb();
    if (pcie1_cfg_read(5U, 0U, 0U, 0x10U) != bar->config_lo ||
        pcie1_cfg_read(5U, 0U, 0U, 0x14U) != bar->config_hi)
        return false;
    pcie1_cfg_write(5U, 0U, 0U, 4U, command & 0xFFFFU);
    return (pcie1_cfg_read(5U, 0U, 0U, 4U) & 7U) == 2U;
}

static const char *quarantine(void)
{
    gic_disable_irq(B50_ROOT_MSI_INTID);
    guc_boot_abort();
    forcewake_drop();
    bool revoked = revoke_bme();
    if (native.attached) {
        (void)pcie1_containment_quarantine(&containment, &authority.endpoint, 0U,
                                          PCIE1_CONTAINMENT_FAULT_AER);
        (void)pcie1_bar_lease_revoke(&lease, &authority.bar, 0U,
                                    PCIE1_BAR_LEASE_FAULT_FAILURE);
        native.attached = 0U;
    }
    return revoked ? "b50 quarantined; BME revoked; cold recovery required\n" :
                     "b50 quarantined; BME revocation UNCONFIRMED; cold recovery required\n";
}
#endif

static void b50_native_service_impl(void)
{
#if PIOS_HAS_PCIE1
    if (core_id() || !native.initialized)
        return;
    if (native.firmware_handle != ADRV_HANDLE_INVALID) {
        u32 reason = ADRV_REASON_NONE;
        if (adrv_take_result(native.firmware_handle, &reason)) {
            native.firmware_handle = ADRV_HANDLE_INVALID;
            if (reason == ADRV_REASON_DONE) {
                native.firmware_state = B50_FW_STATE_STAGED;
                native.firmware_failure = 0U;
            } else {
                native.firmware_failure = reason;
                uart_puts(quarantine());
            }
        }
    }
    if (native.forcewake_handle != ADRV_HANDLE_INVALID) {
        u32 reason = ADRV_REASON_NONE;
        if (adrv_take_result(native.forcewake_handle, &reason)) {
            native.forcewake_handle = ADRV_HANDLE_INVALID;
            if (reason == ADRV_REASON_DONE) {
                native.forcewake_state = B50_FWK_DONE;
                native.forcewake_failure = 0U;
            } else {
                native.forcewake_failure = reason;
                uart_puts(quarantine());
            }
        }
    }
    if (native.ggtt_handle != ADRV_HANDLE_INVALID) {
        u32 reason = ADRV_REASON_NONE;
        if (adrv_take_result(native.ggtt_handle, &reason)) {
            native.ggtt_handle = ADRV_HANDLE_INVALID;
            if (reason == ADRV_REASON_DONE) {
                native.ggtt_state = B50_GGTT_DONE;
                native.ggtt_failure = 0U;
            } else {
                native.ggtt_failure = reason;
                ggtt_diag.ggtt_framework_reason = reason;
                uart_puts(quarantine());
            }
        }
    }
    if (boot_prep.handle != ADRV_HANDLE_INVALID) {
        u32 reason = ADRV_REASON_NONE;
        if (adrv_take_result(boot_prep.handle, &reason)) {
            boot_prep.handle = ADRV_HANDLE_INVALID;
            if (reason == ADRV_REASON_DONE) {
                boot_prep.state = B50_PREP_DONE;
                boot_prep.failure = 0U;
            } else {
                boot_prep.failure = reason;
                ggtt_diag.prep_framework_reason = reason;
                uart_puts(quarantine());
            }
        }
    }
    if (guc_boot.handle != ADRV_HANDLE_INVALID) {
        u32 reason = ADRV_REASON_NONE;
        if (adrv_take_result(guc_boot.handle, &reason)) {
            guc_boot.handle = ADRV_HANDLE_INVALID;
            if (reason == ADRV_REASON_DONE && guc_boot.ready) {
                guc_boot.state = B50_BOOT_DONE;
                guc_boot.failure = 0U;
            } else {
                guc_boot.failure = reason;
                ggtt_diag.boot_framework_reason = reason;
                boot_diag.framework_reason = reason;
                boot_diag.failure = guc_boot.failure;
                guc_boot_abort();
                uart_puts(quarantine());
            }
        }
    }
    if (ct_boot.handle != ADRV_HANDLE_INVALID) {
        u32 reason = ADRV_REASON_NONE;
        if (adrv_take_result(ct_boot.handle, &reason)) {
            ct_boot.handle = ADRV_HANDLE_INVALID;
            if (reason == ADRV_REASON_DONE &&
                ct_boot.state == B50_CT_DONE) {
                ct_boot.failure = 0U;
            } else {
                ct_boot.failure = reason;
                guc_boot_abort();
                uart_puts(quarantine());
            }
        }
    }
    if (context_boot.handle != ADRV_HANDLE_INVALID) {
        u32 reason = ADRV_REASON_NONE;
        if (adrv_take_result(context_boot.handle, &reason)) {
            context_boot.handle = ADRV_HANDLE_INVALID;
            if (reason == ADRV_REASON_DONE &&
                context_boot.state == B50_CONTEXT_DONE) {
                context_boot.failure = 0U;
            } else {
                context_boot.failure = reason;
                guc_boot_abort();
                uart_puts(quarantine());
            }
        }
    }
    if (submit_boot.handle != ADRV_HANDLE_INVALID) {
        u32 reason = ADRV_REASON_NONE;
        if (adrv_take_result(submit_boot.handle, &reason)) {
            submit_boot.handle = ADRV_HANDLE_INVALID;
            if (reason == ADRV_REASON_DONE &&
                submit_boot.state == B50_SUBMIT_DONE) {
                submit_boot.failure = 0U;
            } else {
                submit_boot.failure = reason;
                guc_boot_abort();
                uart_puts(quarantine());
            }
        }
    }
    u64 sequence = trigger.sequence;
    dmb();
    if (sequence == continuation.acknowledged)
        return;
    if (timer_monotonic_ms() >= trigger.deadline) {
        uart_puts(quarantine());
        continuation.acknowledged = sequence;
        return;
    }
    if (continuation.queued != sequence &&
        airq_post_from(0U, AIRQ_SRC_PCIE1_MSI, 0U))
        continuation.queued = sequence;
#endif
}

static bool b50_native_blocks_legacy_impl(void)
{
#if PIOS_HAS_PCIE1
    return native.initialized != 0U;
#else
    return false;
#endif
}

static const char *b50_native_command_impl(u32 operation)
{
    if (core_id() != 0U)
        return "b50 rejected: core-0 owner only\n";
#if !PIOS_HAS_PCIE1
    (void)operation;
    return "b50 rejected: platform has no PCIe1\n";
#else
    if (operation == B50_NATIVE_VALIDATE) {
        bool valid = live_valid();
        if (valid)
            return "b50 validation PASS stage=110\n";
        switch (validation_stage) {
        case 10U: return "b50 validation FAIL stage=10 root-link\n";
        case 20U: return "b50 validation FAIL stage=20 topology\n";
        case 30U: case 31U: case 32U: case 33U:
        case 34U: case 35U: case 36U:
            return "b50 validation FAIL stage=3x path-identity-command-route-aer\n";
        case 40U: return "b50 validation FAIL stage=40 external-gen3\n";
        case 50U: return "b50 validation FAIL stage=50 bar-selection-map\n";
        case 60U: return "b50 validation FAIL stage=60 root-window-registers\n";
        case 70U: return "b50 validation FAIL stage=70 inbound-size-control\n";
        case 80U: return "b50 validation FAIL stage=80 root-aer-or-nc\n";
        case 90U: return "b50 validation FAIL stage=90 bar-device-map-or-deadline\n";
        case 100U: return "b50 validation FAIL stage=100 gmd-id\n";
        default: return "b50 validation FAIL stage=110 final-deadline\n";
        }

    }
    if (operation == B50_NATIVE_FIRMWARE) {
        struct bmg_guc_artifact artifact;
        if (!bmg_guc_artifact_get(&artifact))
            return "b50 GuC rejected: embedded artifact hash/CSS policy failed; BME remains off\n";
        if (artifact.info.release.major != 70U ||
            artifact.info.release.minor != 72U ||
            artifact.info.release.patch != 1U ||
            artifact.info.submission.major != 1U ||
            artifact.info.submission.minor != 38U ||
            artifact.info.submission.patch != 1U ||
            artifact.info.private_data_bytes != 9441280U)
            return "b50 GuC rejected: embedded metadata changed; BME remains off\n";
        return "b50 GuC v70.72.1 submission=1.38.1 bytes=385856 private=9441280 SHA256=verified; not uploaded; BME off\n";
    }
    if (operation == B50_NATIVE_STATUS)
        return native.attached && guc_boot.ready ?
            "b50 attached; GuC v70.72.1 READY; contained BME on; CT/submission unproven\n" :
            native.attached && native.firmware_state == B50_FW_STATE_STAGED ?
            "b50 attached; GuC v70.72.1 staged+verified; BME off; GGTT/upload/queue/DMA unproven\n" :
            native.attached ?
            "b50 attached; GuC not staged; BME off; GGTT/upload/queue/DMA unproven\n" :
            "b50 detached; no hardware authority; DMA unproven\n";
    if (operation == B50_NATIVE_REVOKE)
        return quarantine();
    if (!live_valid())
        return quarantine();
    if (operation == B50_NATIVE_ATTACH) {
        struct pcie1_bar_lease_request request = {0};
        if (native.attached)
            return "b50 already attached\n";
        if (native.initialized)
            return "b50 rejected: retired lifecycle; cold recovery required\n";
        native.initialized = 1U;
        gic_disable_irq(B50_ROOT_MSI_INTID);
        if (!airq_register(AIRQ_SRC_PCIE1_MSI, AIRQ_PRIO_NORMAL, 0U,
                            native_bottom, NULL))
            return quarantine();
        irq_register(B50_ROOT_MSI_INTID, native_irq);
        if (!probe_bar(&request.bar))
            return quarantine();
        request.aperture.cpu_base = PIOS_PCIE1_CPU_WIN_BASE;
        request.aperture.pci_base = 0x80000000ULL;
        request.aperture.length = 0x1000000ULL;
        request.aperture.attribute = PCIE1_BAR_LEASE_ATTR_DEVICE_nGnRnE;
        request.flags = (request.bar.config_lo & 8U) ?
            PCIE1_BAR_LEASE_ALLOW_PREFETCHABLE : 0U;
        if (!pcie1_bar_lease_init(&lease, 82U, 0U) ||
            !pcie1_bar_lease_acquire(&lease, 0x500U, 0U, &request, &authority.bar))
            return quarantine();
        native.attached = 1U;
        if (!pcie1_containment_init(&containment, 82U, 0x500U,
                                    authority.bar.generation, 0U, &authority.endpoint))
            return quarantine();
        return "b50 native generation lease attached; BME remains off\n";
    }
    if (!native.attached)
        return "b50 rejected: attach required\n";
    if (operation == B50_NATIVE_PREFLIGHT) {
        if (!owned_slice_check())
            return quarantine();
        return "b50 mapping/identity/aperture and CPU-owned redzones passed; DMA/IRQ unproven\n";
    }
    if (operation == B50_NATIVE_STAGE) {
        if (native.firmware_handle != ADRV_HANDLE_INVALID ||
            native.firmware_state == B50_FW_STATE_STAGED)
            return native.firmware_state == B50_FW_STATE_STAGED ?
                "b50 GuC already staged+verified; BME off\n" :
                "b50 GuC staging already active; BME off\n";
        if (!bmg_guc_artifact_get(&guc_artifact))
            return quarantine();
        native.firmware_offset = 0U;
        native.firmware_state = B50_FW_STATE_COPY;
        native.firmware_failure = 0U;
        native.firmware_handle = adrv_submit(
            "b50-guc-stage", firmware_stage_step, NULL, 1U, 5000U,
            ADRV_CADENCE_FAST);
        if (native.firmware_handle == ADRV_HANDLE_INVALID) {
            native.firmware_state = B50_FW_STATE_IDLE;
            return quarantine();
        }
        return "b50 GuC guarded asynchronous staging scheduled; BME off\n";
    }
    if (operation == B50_NATIVE_FORCEWAKE) {
        if (native.forcewake_handle != ADRV_HANDLE_INVALID)
            return "b50 forcewake proof already active; BME off\n";
        if (native.forcewake_state == B50_FWK_DONE)
            return "b50 GT+Render forcewake acquire/GMD_ID/release already proven; BME off\n";
        native.forcewake_state = B50_FWK_IDLE;
        native.forcewake_gmd_id = 0U;
        native.forcewake_failure = 0U;
        native.forcewake_handle = adrv_submit(
            "b50-forcewake", forcewake_step, NULL, 1U, 500U,
            ADRV_CADENCE_FAST);
        if (native.forcewake_handle == ADRV_HANDLE_INVALID)
            return quarantine();
        return "b50 bounded GT+Render forcewake proof scheduled; BME off\n";
    }
    if (operation == B50_NATIVE_GGTT) {
        if (native.ggtt_handle != ADRV_HANDLE_INVALID)
            return "b50 GGTT firmware/queue mapping already active; BME off\n";
        if (native.ggtt_state == B50_GGTT_DONE)
            return "b50 GGTT firmware+queue PTEs mapped and verified; BME off\n";
        if (native.firmware_state != B50_FW_STATE_STAGED ||
            native.forcewake_state != B50_FWK_DONE)
            return "b50 GGTT rejected: stage and forcewake proof required; BME off\n";
        native.ggtt_state = B50_GGTT_MAP_FW;
        native.ggtt_page = 0U;
        native.ggtt_failure = 0U;
        ggtt_diag = (struct b50_native_diag){.attached = 1U};
        native.ggtt_handle = adrv_submit(
            "b50-ggtt", ggtt_step, NULL, 1U, 10000U, ADRV_CADENCE_FAST);
        if (native.ggtt_handle == ADRV_HANDLE_INVALID) {
            native.ggtt_state = B50_GGTT_IDLE;
            return quarantine();
        }

        return "b50 bounded GGTT firmware+queue mapping scheduled; BME off\n";
    }
    if (operation == B50_NATIVE_PREPARE) {
        if (boot_prep.handle != ADRV_HANDLE_INVALID)
            return "b50 minimal ADS/log preparation already active; BME off\n";
        if (boot_prep.state == B50_PREP_DONE)
            return "b50 minimal ADS/log/params prepared+guarded; BME off\n";
        if (native.ggtt_state != B50_GGTT_DONE)
            return "b50 prepare rejected: verified GGTT mapping required; BME off\n";
        boot_prep.state = B50_PREP_ZERO_ADS;
        boot_prep.offset = 0U;
        boot_prep.failure = 0U;
        ggtt_diag.prep_state = B50_PREP_ZERO_ADS;
        ggtt_diag.prep_offset = 0U;
        ggtt_diag.prep_detail = B50_PREP_DIAG_NONE;
        ggtt_diag.prep_framework_reason = ADRV_REASON_NONE;
        boot_prep.revision = pcie1_cfg_read(5U, 0U, 0U, 8U) & 0xFFU;
        boot_prep.handle = adrv_submit(
            "b50-guc-prep", boot_prepare_step, NULL, 1U, 30000U,
            ADRV_CADENCE_FAST);
        if (boot_prep.handle == ADRV_HANDLE_INVALID) {
            boot_prep.state = B50_PREP_IDLE;
            return quarantine();
        }
        return "b50 bounded minimal ADS/log/params preparation scheduled; BME off\n";
    }
    if (operation == B50_NATIVE_BOOT) {
        if (guc_boot.handle != ADRV_HANDLE_INVALID)
            return "b50 contained GuC boot already active\n";
        if (guc_boot.ready)
            return "b50 GuC v70.72.1 already READY; contained BME on\n";
        if (boot_prep.state != B50_PREP_DONE)
            return "b50 GuC boot rejected: guarded ADS/log preparation required; BME off\n";
        guc_boot = (typeof(guc_boot)){0};
        boot_diag = (struct b50_native_boot_diag){0};
        guc_boot.state = B50_BOOT_ACQUIRE_GT;
        ggtt_diag.boot_state = B50_BOOT_ACQUIRE_GT;
        ggtt_diag.boot_framework_reason = ADRV_REASON_NONE;
        guc_boot.handle = adrv_submit(
            "b50-guc-boot", guc_boot_step, NULL, 1U, 5000U,
            ADRV_CADENCE_FAST);
        if (guc_boot.handle == ADRV_HANDLE_INVALID) {
            guc_boot.state = B50_BOOT_IDLE;
            return quarantine();
        }
        return "b50 contained GuC reset+DMA+ready proof scheduled\n";
    }
    if (operation == B50_NATIVE_CT) {
        if (ct_boot.handle != ADRV_HANDLE_INVALID)
            return "b50 GuC CT bootstrap already active\n";
        if (ct_boot.state == B50_CT_DONE)
            return "b50 GuC CT bidirectional negative-probe proof complete\n";
        if (!guc_boot.ready)
            return "b50 CT rejected: GuC READY required\n";
        ct_boot = (typeof(ct_boot)){0};
        ct_boot.state = B50_CT_ZERO;
        ct_boot.handle = adrv_submit(
            "b50-guc-ct", ct_step, NULL, 1U, 5000U,
            ADRV_CADENCE_FAST);
        if (ct_boot.handle == ADRV_HANDLE_INVALID) {
            ct_boot.state = B50_CT_IDLE;
            return quarantine();
        }
        return "b50 bounded GuC CT bidirectional negative probe scheduled\n";
    }
    if (operation == B50_NATIVE_CONTEXT) {
        if (context_boot.handle != ADRV_HANDLE_INVALID)
            return "b50 GuC CCS0 context registration already active\n";
        if (context_boot.state == B50_CONTEXT_DONE)
            return "b50 GuC CCS0 width-1 context registered\n";
        if (!guc_boot.ready || ct_boot.state != B50_CT_DONE)
            return "b50 context rejected: GuC READY + CT proof required\n";
        context_boot = (typeof(context_boot)){0};
        context_boot.state = B50_CONTEXT_BUILD;
        context_boot.handle = adrv_submit(
            "b50-guc-context", context_step, NULL, 1U, 2000U,
            ADRV_CADENCE_FAST);
        if (context_boot.handle == ADRV_HANDLE_INVALID) {
            context_boot.state = B50_CONTEXT_IDLE;
            return quarantine();
        }
        return "b50 bounded GuC CCS0 width-1 context registration scheduled\n";
    }
    if (operation == B50_NATIVE_SUBMIT) {
        if (submit_boot.handle != ADRV_HANDLE_INVALID)
            return "b50 CCS0 store-only submission already active\n";
        if (submit_boot.state == B50_SUBMIT_DONE)
            return "b50 CCS0 store-only result=42 completion=FEED0001 PASS\n";
        if (context_boot.state != B50_CONTEXT_DONE)
            return "b50 submit rejected: registered context required\n";
        submit_boot = (typeof(submit_boot)){0};
        memset(submit_engine_diag, 0, sizeof(submit_engine_diag));
        submit_boot.state = B50_SUBMIT_ENABLE_SEND;
        submit_boot.handle = adrv_submit(
            "b50-guc-submit", submit_step, NULL, 1U, 2000U,
            ADRV_CADENCE_FAST);
        if (submit_boot.handle == ADRV_HANDLE_INVALID) {
            submit_boot.state = B50_SUBMIT_IDLE;
            return quarantine();
        }
        return "b50 bounded CCS0 store-only submission scheduled\n";
    }
    if (operation == B50_NATIVE_CANARY)
        return guc_boot.ready ?
            "b50 rejected: CT/submission queue and completion IRQ not implemented; GuC ready, contained BME on\n" :
            "b50 rejected: GuC boot and completion proof required; BME remains off\n";
    return "b50 rejected: unknown operation\n";
#endif
}

#define B50_MODULE_OP_COMMAND 1U
#define B50_MODULE_OP_SERVICE 2U
#define B50_MODULE_OP_BLOCKS  3U

static u64 b50_static_dispatch(u32 op, u64 a0, u64 a1)
{
    (void)a1;
    if (op == B50_MODULE_OP_COMMAND)
        return (u64)(usize)b50_native_command_impl((u32)a0);
    if (op == B50_MODULE_OP_SERVICE) {
        b50_native_service_impl();
        return 0U;
    }
    if (op == B50_MODULE_OP_BLOCKS)
        return b50_native_blocks_legacy_impl() ? 1U : 0U;
    return 0U;
}

const char *b50_native_command(u32 operation)
{
    return (const char *)(usize)module_dispatch_call(
        MODULE_ID_B50, B50_MODULE_OP_COMMAND, operation, 0U,
        b50_static_dispatch);
}

void b50_native_service(void)
{
    (void)module_dispatch_call(
        MODULE_ID_B50, B50_MODULE_OP_SERVICE, 0U, 0U,
        b50_static_dispatch);
}

bool b50_native_blocks_legacy(void)
{
    return module_dispatch_call(
        MODULE_ID_B50, B50_MODULE_OP_BLOCKS, 0U, 0U,
        b50_static_dispatch) != 0U;
}

void b50_native_diag(struct b50_native_diag *out)
{
    if (!out)
        return;
#if PIOS_HAS_PCIE1
    ggtt_diag.attached = native.attached;
    ggtt_diag.ggtt_state = native.ggtt_state;
    ggtt_diag.ggtt_page = native.ggtt_page;
    ggtt_diag.prep_state = boot_prep.state;
    ggtt_diag.prep_offset = boot_prep.offset;
    ggtt_diag.boot_state = guc_boot.state;
    dmb();
    *out = ggtt_diag;
#else
    *out = (struct b50_native_diag){0};
#endif
}

void b50_native_boot_diag(struct b50_native_boot_diag *out)
{
    if (!out)
        return;
#if PIOS_HAS_PCIE1
    boot_diag.state = guc_boot.state;
    boot_diag.failure = guc_boot.failure;
    boot_diag.ready = guc_boot.ready;
    boot_diag.last_status = guc_boot.last_status;
    boot_diag.dma_ctrl = guc_boot.dma_ctrl;
    dmb();
    *out = boot_diag;
#else
    *out = (struct b50_native_boot_diag){0};
#endif
}

void b50_native_ct_diag(struct b50_native_ct_diag *out)
{
    if (!out)
        return;
#if PIOS_HAS_PCIE1
    u8 *base = ct_cpu_base();
    const struct intel_guc_ct_desc *h2g =
        (const struct intel_guc_ct_desc *)(const void *)
        (base + (B50_CT_H2G_DESC_OFFSET - B50_CT_BASE_OFFSET));
    const struct intel_guc_ct_desc *g2h =
        (const struct intel_guc_ct_desc *)(const void *)
        (base + (B50_CT_G2H_DESC_OFFSET - B50_CT_BASE_OFFSET));
    const u32 *hwconfig = (const u32 *)(const void *)
        (base + (B50_CT_HWCONFIG_OFFSET - B50_CT_BASE_OFFSET));
    *out = (struct b50_native_ct_diag){
        .state = ct_boot.state,
        .index = ct_boot.index,
        .retries = ct_boot.retries,
        .response = ct_boot.response,
        .failure = ct_boot.failure,
        .h2g_head = h2g->head,
        .h2g_tail = h2g->tail,
        .h2g_status = h2g->status,
        .g2h_head = g2h->head,
        .g2h_tail = g2h->tail,
        .g2h_status = g2h->status,
        .hwconfig0 = hwconfig[0],
        .context_state = context_boot.state,
        .context_response = context_boot.response,
        .context_failure = context_boot.failure,
        .context_ring_tail = context_boot.ring_tail
    };
#else
    *out = (struct b50_native_ct_diag){0};
#endif
}

void b50_native_submit_diag(struct b50_native_submit_diag *out)
{
    if (!out)
        return;
#if PIOS_HAS_PCIE1
    u8 *queue = ct_cpu_base() - B50_CT_BASE_OFFSET;
    u8 *ct_base = queue + B50_CT_BASE_OFFSET;
    const struct intel_guc_ct_desc *g2h =
        (const struct intel_guc_ct_desc *)(const void *)
        (ct_base + (B50_CT_G2H_DESC_OFFSET - B50_CT_BASE_OFFSET));
    dmb();
    *out = (struct b50_native_submit_diag){
        .state = submit_boot.state,
        .response = submit_boot.response,
        .failure = submit_boot.failure,
        .result = submit_boot.result,
        .completion = submit_boot.completion,
        .g2h_head = g2h->head,
        .g2h_tail = g2h->tail,
        .g2h_status = g2h->status,
        .engine_head = submit_engine_diag[0],
        .engine_tail = submit_engine_diag[1],
        .engine_start = submit_engine_diag[2],
        .engine_control = submit_engine_diag[3],
        .engine_hws_pga = submit_engine_diag[4],
        .engine_context_control = submit_engine_diag[5],
        .engine_acthd = submit_engine_diag[6],
        .engine_ipehr = submit_engine_diag[7]
    };
#else
    *out = (struct b50_native_submit_diag){0};
#endif
}
