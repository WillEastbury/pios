/*
 * cyw43.c - Broadcom CYW43 WiFi FullMAC driver
 *
 * Broadcom/Cypress combo chips on Pi 5 and BCM2837-family boards, via the
 * board-native SDIO host (not RP1 -- see sdio.h).
 * FullMAC: firmware handles 802.11/WPA2, host speaks SDPCM/BCDC.
 *
 * Architecture:
 *   SDIO func 0: Common I/O Area (CCCR) — card management
 *   SDIO func 1: Silicon Backplane — register/RAM access
 *   SDIO func 2: WLAN data — Ethernet frames
 *
 * Backplane access: CMD52 sets a 32KB window, CMD53 moves data.
 * SDPCM: framing protocol over func 2 — control + data channels.
 * BCDC:  control messages within SDPCM control channel.
 *
 * Reference: Linux drivers/net/wireless/broadcom/brcm80211/brcmfmac/
 *            Circle WiFi (bcm4343.cpp, hostap.cpp)
 */

#include "cyw43.h"
#include "sdio.h"
#include "fat32.h"
#include "watchdog.h"
#include "uart.h"
#include "timer.h"
#include "fb.h"
#include "crypto.h"
#include "exception.h"
#include "board_detect.h"
#include "mailbox.h"

/* ── Constants ── */

/* SDIO func 1 backplane registers (CMD52 addresses within func 1) */
#define SB_INT_STATUS           0x20
#define SB_INT_HOST_MASK        0x24
#define SB_FUNC_INT_MASK        0x34
#define SB_TO_SB_MBOX           0x40
#define SB_TO_SB_MBOX_DATA      0x48
#define SB_TO_HOST_MBOX_DATA    0x4C

/* Backplane window granularity */
#define BACKPLANE_WIN_SIZE      0x8000  /* 32KB */
#define BACKPLANE_WIN_MASK      0x7FFF
#define BACKPLANE_ADDR_MASK     0xFFFFF8000ULL

/* Core enumeration (for AXI cores) */
#define CORE_CTRL               0x0408
#define CORE_RESET_CTRL         0x0800
#define CORE_RESET_STATUS       0x0804
#define CORE_IOCTRL             0x0408
#define CORE_RESETCTRL          0x0800

/* IOCTRL bits */
#define SICF_FGC                0x0002  /* Force gated clocks on */
#define SICF_CLOCK_EN           0x0001  /* Force HT request */
#define SICF_CPUHALT            0x0020  /* ARM core halt */
#define D11_PHYRESET            0x0008
#define D11_PHYCLOCKEN          0x0004

/* RESETCTRL bits */
#define AIRC_RESET              0x0001

/* SOCSRAM wrapper */
#define SOCSRAM_BANKX_IDX       0x10
#define SOCSRAM_BANKX_INFO      0x40
#define SOCSRAM_BANKX_PDA       0x44
#define SOCSRAM_COREINFO        0x00

/* ARM CR4 TCM sizing registers. */
#define ARMCR4_CAP              0x04
#define ARMCR4_BANKIDX          0x40
#define ARMCR4_BANKINFO         0x44
#define ARMCR4_TCBANB_MASK      0x0F
#define ARMCR4_TCBBNB_MASK      0xF0
#define ARMCR4_TCBBNB_SHIFT     4
#define ARMCR4_BSZ_MASK         0x7F
#define ARMCR4_BLK_1K_MASK      0x200
#define ARMCR4_BSZ_MULT         8192U

/* Clock control status register (SDIO func 1) */
#define SDIO_CLKCSR             0x1000E
#define SDIO_PULLUPS            0x1000F
#define SDIO_RFRAMEBC_LO        0x1001B
#define SDIO_RFRAMEBC_HI        0x1001C
#define SDIO_SLEEPCSR           0x1001F
#define SLEEPCSR_KSO            (1U << 0)
#define SLEEPCSR_DEVON          (1U << 1)
#define CLKCSR_ForceALP         0x01
#define CLKCSR_ForceHT          0x02
#define CLKCSR_ReqALP           0x08
#define CLKCSR_ReqHT            0x10
#define CLKCSR_Nohwreq          0x20
#define CLKCSR_ALPavail         0x40
#define CLKCSR_HTavail          0x80

/* Chipcommon GPIO pull registers */
#define CC_GPIOPULLUP           0x58
#define CC_GPIOPULLDOWN         0x5C

/* SDIOD core register offsets */
#define SDIOD_INTSTATUS         0x20
#define SDIOD_INTMASK           0x24
#define SDIOD_TOSBMAILBOX       0x40
#define SDIOD_TOSBMAILBOXDATA   0x48
#define SDIOD_TOHOSTMAILBOXDATA 0x4C
#define SDIOD_I_HMB_FRAME_IND   (1U << 6)
#define SDIOD_I_HMB_HOST_INT    (1U << 7)
#define SDIOD_SMB_INT_ACK       (1U << 1)
#define SDIOD_HMB_DEVREADY      (1U << 1)
#define SDIOD_HMB_FWREADY       (1U << 3)

/* Firmware upload block size */
#define FW_UPLOAD_BLKSZ         64
#define SDIO_FUNC1_BLKSZ        64
#define SDIO_FUNC2_BLKSZ        512
#define CYW_F1_BATCH_BLOCKS     64U
#define CYW_F1_BATCH_BYTES      (CYW_F1_BATCH_BLOCKS * SDIO_FUNC1_BLKSZ)

/* SDPCM frame tags */
#define SDPCM_FRAMETAG_LEN_MASK 0xFFFF

/* Maximum frame size */
#define CYW_MAX_FRAME           4096
/* Must stay comfortably below the ~15 s hardware watchdog so the stall is
 * reported by us, with diagnostics, rather than by a silent reset. */
#define CYW_TX_CREDIT_STALL_MS  8000ULL
#define CYW_SCAN_QUERY_BYTES    416U
#define CYW_BCDC_GET_TIMEOUT_MS 5000ULL
/* Bounded frames consumed per reactor poll pass. */
#define CYW_POLL_BURST_FRAMES   32U
/* Bounded RF-neutral bus pokes issued while a scan is running. */
#define CYW_SCAN_KICK_INTERVAL_MS 250ULL
#define CYW_SCAN_KICKS_MAX        60U
#define CYW_DATA_RX_QUEUE_DEPTH   CYW_POLL_BURST_FRAMES

/* ── State ── */

static u8 cyw_mac[CYW_MAC_LEN];
static u32 cyw_link;
static u8 cyw_tx_seq;
static u8 cyw_tx_max;
static u8 cyw_tx_fcmask;
static u32 cyw_backplane_window;
static bool sdpcm_frame_pending;
static u32 sdpcm_next_len;

/* RX/TX buffers — 64-byte aligned for SDIO DMA compatibility */
static u8 cyw_tx_buf[CYW_MAX_FRAME] ALIGNED(64);
static u8 cyw_rx_buf[CYW_MAX_FRAME] ALIGNED(64);
static u8 cyw_eapol_frame[CYW_EAPOL_MAX] ALIGNED(64);
static u32 cyw_eapol_len;
static u8 cyw_m2_frame[CYW_EAPOL_MAX] ALIGNED(64);
static u32 cyw_m2_len;
struct cyw_data_rx_slot {
    u32 len;
    u8 data[CYW_MAX_FRAME];
};
static struct cyw_data_rx_slot
    cyw_data_rx_queue[CYW_DATA_RX_QUEUE_DEPTH] ALIGNED(64);
static u32 cyw_data_rx_head;
static u32 cyw_data_rx_tail;
static u32 cyw_data_rx_count;

/* BCDC request ID counter */
static u16 bcdc_reqid;
#define BCDC_RESPONSE_SLOTS     4U
#define BCDC_RESPONSE_DATA_MAX  512U
struct bcdc_cached_response {
    bool valid;
    u16 id;
    u16 data_len;
    u32 status;
    u8 data[BCDC_RESPONSE_DATA_MAX];
} ALIGNED(64);
static struct bcdc_cached_response bcdc_responses[BCDC_RESPONSE_SLOTS];
static u16 bcdc_ignore_through;

/* Scan results */
static struct cyw_scan_result scan_results[CYW_MAX_SCAN_RESULTS];
static u32 scan_count;
static bool scan_in_progress;
static bool scan_results_pending;
static u64 scan_ready_ms;
static bool scan_result_request_pending;
static u16 scan_result_request_id;
static u64 scan_result_request_deadline_ms;
static u32 scan_result_request_attempts;
static u32 scan_kicks_remaining;
static u64 scan_next_kick_ms;
static u32 join_kicks_remaining;
static u64 join_next_kick_ms;
static u8 scan_result_buf[CYW_SCAN_QUERY_BYTES] ALIGNED(64);
static struct {
    bool valid;
    u8 bssid[CYW_MAC_LEN];
    u16 chanspec;
} join_target;

static u16 cyw_rsn_caps;
static bool cyw_rsn_caps_override;
static bool cyw_rx_probe_enabled = true;
/* Speculative-probe pacing. `armed` latches a decision across the two
 * pending_bytes() calls that a single frame read performs. */
static bool cyw_rx_probe_armed;
static u32 cyw_rx_probe_backoff_ms;
static u64 cyw_rx_probe_next_ms;
#define CYW_RX_PROBE_BACKOFF_MAX_MS 1000U
static u64 cyw_poll_idle_next_ms;
static bool cyw_rx_irq_hint;

struct cyw_wpa_host {
    bool nonce_ready;
    bool enabled;
    bool m2_sent;
    bool keys_installed;
    u8 pmk[32];
    u8 snonce[32];
    u8 anonce[32];
    u8 ptk[64];
    u8 ap_mac[CYW_MAC_LEN];
    u8 rsn_ie[66];
    u8 rsn_ie_len;
    u64 replay;
};
static struct cyw_wpa_host wpa_host;

/* Discovered core addresses (from EROM scan) */
static u32 cyw_arm_ctl;
static u32 cyw_arm_regs;
static u32 cyw_arm_core_id;
static u32 cyw_d11_ctl;
static u32 cyw_sram_ctl;
static u32 cyw_sram_regs;
static u32 cyw_sram_rev;
static u32 cyw_sdio_regs;
static u32 cyw_ram_base = CYW_RAM_BASE;  /* default, updated from EROM */
static u32 cyw_ram_bytes;
static u16 cyw_chip_id;
static struct cyw43_diag cyw_diag ALIGNED(64);
static struct cyw_event_history cyw_event_history ALIGNED(64);
static cyw43_progress_fn cyw_progress_hook;

static u32 cyw43_board_model(void)
{
#if PIOS_PLATFORM == PIOS_PLATFORM_PI3
    static volatile u32 board_mbox[7] ALIGNED(16);
    board_mbox[0] = sizeof(board_mbox);
    board_mbox[1] = 0U;
    board_mbox[2] = TAG_GET_BOARD_REV;
    board_mbox[3] = 4U;
    board_mbox[4] = 0U;
    board_mbox[5] = 0U;
    board_mbox[6] = TAG_END;
    if (mbox_call(MBOX_CH_PROP, board_mbox) &&
        (board_mbox[4] & 0x80000000U))
        return board_model_from_revision(board_mbox[5]);
#endif
    return BOARD_MODEL_UNKNOWN;
}

/* Pre-loaded blobs (loaded before cyw43_init disturbs SD) */
#define CYW_FW_MAX_SIZE   (700 * 1024)
#define CYW_NVRAM_MAX     4096
#define CYW_CLM_MAX       16384
static u8 fw_buf[CYW_FW_MAX_SIZE] ALIGNED(64);
static u32 fw_buf_len;
static const u8 *fw_data = fw_buf;
static u8 nvram_buf[CYW_NVRAM_MAX];
static u32 nvram_buf_len;
static const u8 *nvram_data = nvram_buf;
static u8 clm_buf[CYW_CLM_MAX];
static u32 clm_buf_len;
static const u8 *clm_data = clm_buf;
static bool blobs_loaded;

/* ── Backplane access ── */

static bool bp_read32(u32 addr, u32 *val);

static bool bp_set_window(u32 addr)
{
    u32 win = addr & (u32)BACKPLANE_ADDR_MASK;
    if (win == cyw_backplane_window)
        return true;

    /* Write 3 bytes of the window address to SB window register */
    if (!sdio_cmd52_write(SDIO_FUNC_BACKPLANE, CYW_BAK_WIN_ADDR + 0,
                          (u8)((win >> 8) & 0xFF)))
        return false;
    if (!sdio_cmd52_write(SDIO_FUNC_BACKPLANE, CYW_BAK_WIN_ADDR + 1,
                          (u8)((win >> 16) & 0xFF)))
        return false;
    if (!sdio_cmd52_write(SDIO_FUNC_BACKPLANE, CYW_BAK_WIN_ADDR + 2,
                          (u8)((win >> 24) & 0xFF)))
        return false;

    cyw_backplane_window = win;
    return true;
}

void cyw43_diag_snapshot(struct cyw43_diag *out)
{
    if (!out)
        return;
    *out = cyw_diag;
}

void cyw43_event_history_snapshot(struct cyw_event_history *out)
{
    if (!out)
        return;
    *out = cyw_event_history;
}

/* True while WiFi has work in flight and therefore justifies the fast reactor
 * poll cadence. Reads owned state only: safe from the timer tick. Polling SDIO
 * at the fast cadence when the radio is idle burns core 0 for nothing. */
bool cyw43_poll_busy(void)
{
    return scan_in_progress || scan_result_request_pending ||
           cyw_link == CYW_LINK_JOINING;
}

bool cyw43_install_blob(u32 kind, const u8 *data, u32 len)
{
    if (!data || len == 0U)
        return false;
    if (kind == CYW_BLOB_FIRMWARE) {
        if (len > sizeof(fw_buf))
            return false;
        memcpy(fw_buf, data, len);
        fw_data = fw_buf;
        fw_buf_len = len;
    } else if (kind == CYW_BLOB_NVRAM) {
        if (len > sizeof(nvram_buf))
            return false;
        memcpy(nvram_buf, data, len);
        nvram_data = nvram_buf;
        nvram_buf_len = len;
    } else if (kind == CYW_BLOB_CLM) {
        if (len > sizeof(clm_buf))
            return false;
        memcpy(clm_buf, data, len);
        clm_data = clm_buf;
        clm_buf_len = len;
    } else {
        return false;
    }
    blobs_loaded = fw_buf_len != 0U && nvram_buf_len != 0U &&
                   clm_buf_len != 0U;
    cyw_diag.fw_len = fw_buf_len;
    cyw_diag.nvram_len = nvram_buf_len;
    cyw_diag.clm_len = clm_buf_len;
    return true;
}

bool cyw43_blobs_ready(void)
{
    return blobs_loaded;
}

bool cyw43_runtime_ready(void)
{
    return cyw_diag.stage >= 25U && cyw_diag.func2_ready != 0U;
}

bool cyw43_d11_state(u32 *resetctrl, u32 *ioctrl)
{
    if (!resetctrl || !ioctrl || cyw_d11_ctl == 0U)
        return false;
    return bp_read32(cyw_d11_ctl + CORE_RESETCTRL, resetctrl) &&
           bp_read32(cyw_d11_ctl + CORE_IOCTRL, ioctrl);
}

void cyw43_set_progress_hook(cyw43_progress_fn hook)
{
    cyw_progress_hook = hook;
}

u32 cyw43_ram_size(void)
{
    return cyw_ram_bytes;
}

/* CMD52-based 4-byte backplane read — avoids DAT line entirely */
static bool bp_read32(u32 addr, u32 *val)
{
    if (!bp_set_window(addr)) {
        uart_puts("[bpr] win fail @");
        uart_hex(addr);
        uart_puts("\n");
        return false;
    }

    u32 off = addr & BACKPLANE_WIN_MASK;
    u8 b0, b1, b2, b3;
    if (!sdio_cmd52_read(SDIO_FUNC_BACKPLANE, (off | 0x8000), &b0)) {
        uart_puts("[bpr] r52 fail off=");
        uart_hex(off);
        uart_puts("\n");
        return false;
    }
    sdio_cmd52_read(SDIO_FUNC_BACKPLANE, (off | 0x8000) + 1, &b1);
    sdio_cmd52_read(SDIO_FUNC_BACKPLANE, (off | 0x8000) + 2, &b2);
    sdio_cmd52_read(SDIO_FUNC_BACKPLANE, (off | 0x8000) + 3, &b3);
    *val = (u32)b0 | ((u32)b1 << 8) | ((u32)b2 << 16) | ((u32)b3 << 24);
    return true;
}

/* CMD53 word-mode 4-byte backplane write — required for TCM RAM (CMD52 byte
 * writes only work for register space; RAM enforces 4-byte atomic access). */
static bool bp_write32(u32 addr, u32 val)
{
    if (!bp_set_window(addr)) {
        uart_puts("[bpw] win fail @");
        uart_hex(addr);
        uart_puts("\n");
        return false;
    }

    u32 off = addr & BACKPLANE_WIN_MASK;
    u8 buf[4] = {
        (u8)(val & 0xFF),
        (u8)((val >> 8) & 0xFF),
        (u8)((val >> 16) & 0xFF),
        (u8)((val >> 24) & 0xFF),
    };
    if (!sdio_cmd53_write(SDIO_FUNC_BACKPLANE, off | 0x8000, buf, 4, true)) {
        uart_puts("[bpw] c53w4 fail addr=");
        uart_hex(addr);
        uart_puts("\n");
        return false;
    }
    return true;
}

static bool bp_read_buf(u32 addr, u8 *buf, u32 len)
{
    while (len > 0) {
        if (!bp_set_window(addr))
            return false;

        u32 off = addr & BACKPLANE_WIN_MASK;
        u32 chunk = BACKPLANE_WIN_SIZE - off;
        if (chunk > len) chunk = len;
        if (chunk > SDIO_FUNC1_BLKSZ) chunk = SDIO_FUNC1_BLKSZ;

        if (!sdio_cmd53_read(SDIO_FUNC_BACKPLANE, off | 0x8000,
                             buf, chunk, true))
            return false;

        addr += chunk;
        buf += chunk;
        len -= chunk;
    }
    return true;
}

static bool bp_write_buf(u32 addr, const u8 *buf, u32 len)
{
    if (addr & 3) {
        uart_puts("[bpwb] unaligned addr=");
        uart_hex(addr);
        uart_puts("\n");
        return false;
    }

    while (len > 0U) {
        if (!bp_set_window(addr))
            return false;
        u32 off = addr & BACKPLANE_WIN_MASK;
        u32 window_left = BACKPLANE_WIN_SIZE - off;
        u32 full = len & ~(SDIO_FUNC1_BLKSZ - 1U);
        if (full > window_left)
            full = window_left & ~(SDIO_FUNC1_BLKSZ - 1U);
        if (full > CYW_F1_BATCH_BYTES)
            full = CYW_F1_BATCH_BYTES;

        if (full != 0U) {
            u32 nblks = full / SDIO_FUNC1_BLKSZ;
            if (!sdio_cmd53_write_blocks(SDIO_FUNC_BACKPLANE,
                                         off | 0x8000U, buf,
                                         SDIO_FUNC1_BLKSZ, nblks, true))
                return false;
            addr += full;
            buf += full;
            len -= full;
        } else {
            u32 chunk = len < window_left ? len : window_left;
            if (chunk > SDIO_FUNC1_BLKSZ)
                chunk = SDIO_FUNC1_BLKSZ;
            if (!sdio_cmd53_write(SDIO_FUNC_BACKPLANE,
                                  off | 0x8000U, buf, chunk, true))
                return false;
            addr += chunk;
            buf += chunk;
            len -= chunk;
        }
        if ((addr & 0xFFFFU) == 0U) {
            uart_puts("[bpwb] @");
            uart_hex(addr);
            uart_puts("\n");
        }
        watchdog_hw_pet();
        if (cyw_progress_hook)
            cyw_progress_hook();
    }
    return true;
}

static bool probe_f1_multiblock(void)
{
    static u8 write_buf[CYW_F1_BATCH_BYTES] ALIGNED(64);
    static u8 read_buf[CYW_F1_BATCH_BYTES] ALIGNED(64);
    for (u32 i = 0U; i < sizeof(write_buf); i++)
        write_buf[i] = (u8)(0xA5U ^ i ^ (i >> 8));
    memset(read_buf, 0, sizeof(read_buf));

    if (!bp_set_window(cyw_ram_base))
        return false;
    u32 off = (cyw_ram_base & BACKPLANE_WIN_MASK) | 0x8000U;
    if (!sdio_cmd53_write_blocks(SDIO_FUNC_BACKPLANE, off, write_buf,
                                 SDIO_FUNC1_BLKSZ,
                                 CYW_F1_BATCH_BLOCKS, true))
        return false;
    if (!sdio_cmd53_read_blocks(SDIO_FUNC_BACKPLANE, off, read_buf,
                                SDIO_FUNC1_BLKSZ,
                                CYW_F1_BATCH_BLOCKS, true))
        return false;
    return memcmp(write_buf, read_buf, sizeof(write_buf)) == 0;
}

/* ── Core control ── */

static bool core_disable(u32 core_base, u32 prereset, u32 reset)
{
    u32 val;

    /* Check if already in reset */
    if (!bp_read32(core_base + CORE_RESETCTRL, &val))
        return false;
    if ((val & AIRC_RESET) == 0U) {
        /* Match brcmf_chip_ai_coredisable: preserve required pre-reset flags
         * while forcing clocks, assert reset, then configure the in-reset state. */
        if (!bp_write32(core_base + CORE_IOCTRL,
                        prereset | SICF_FGC | SICF_CLOCK_EN))
            return false;
        if (!bp_read32(core_base + CORE_IOCTRL, &val))
            return false;
        delay_cycles(1000);

        if (!bp_write32(core_base + CORE_RESETCTRL, AIRC_RESET))
            return false;
        for (u32 i = 0; i < 50U; i++) {
            if (bp_read32(core_base + CORE_RESETCTRL, &val) &&
                (val & AIRC_RESET))
                break;
            timer_delay_us(50U);
        }
        if (!bp_read32(core_base + CORE_RESETCTRL, &val) ||
            (val & AIRC_RESET) == 0U)
            return false;
    }

    return bp_write32(core_base + CORE_IOCTRL,
                      reset | SICF_FGC | SICF_CLOCK_EN);
}

static bool core_reset(u32 core_base, u32 prereset,
                       u32 reset, u32 postreset)
{
    if (!core_disable(core_base, prereset, reset))
        return false;

    u32 val;
    for (u32 i = 0; i < 50U; i++) {
        if (!bp_read32(core_base + CORE_RESETCTRL, &val))
            return false;
        if ((val & AIRC_RESET) == 0U)
            break;
        if (!bp_write32(core_base + CORE_RESETCTRL, 0U))
            return false;
        timer_delay_us(50U);
    }
    if (!bp_read32(core_base + CORE_RESETCTRL, &val) ||
        (val & AIRC_RESET))
        return false;
    if (!bp_write32(core_base + CORE_IOCTRL,
                    postreset | SICF_CLOCK_EN))
        return false;
    return bp_read32(core_base + CORE_IOCTRL, &val);
}

static u32 cyw43_firmware_ramsize(void)
{
    if (cyw_arm_core_id == 0x82AU) {
        u32 coreinfo = 0U;
        if (!cyw_sram_ctl || !cyw_sram_regs || cyw_sram_rev <= 7U ||
            cyw_sram_rev == 12U ||
            !core_reset(cyw_sram_ctl, 0U, 0U, 0U) ||
            !bp_read32(cyw_sram_regs + SOCSRAM_COREINFO, &coreinfo))
            return 0U;
        u32 banks = (coreinfo >> 4U) & 0x0FU;
        if (banks == 0U || banks > 32U)
            return 0U;
        u32 bytes = 0U;
        for (u32 idx = 0U; idx < banks; idx++) {
            u32 info = 0U;
            if (!bp_write32(cyw_sram_regs + SOCSRAM_BANKX_IDX, idx) ||
                !bp_read32(cyw_sram_regs + SOCSRAM_BANKX_INFO, &info))
                return 0U;
            bytes += ((info & 0x3FU) + 1U) * 8192U;
        }
        cyw_ram_base = 0U;
        if (cyw_chip_id == CYW43430_CHIP_ID) {
            if (!bp_write32(cyw_sram_regs + SOCSRAM_BANKX_IDX, 3U) ||
                !bp_write32(cyw_sram_regs + SOCSRAM_BANKX_PDA, 0U))
                return 0U;
        }
        uart_puts("[cyw] SOCRAM banks=");
        uart_hex(banks);
        uart_puts(" bytes=");
        uart_hex(bytes);
        uart_puts("\n");
        return bytes;
    }

    u32 cap = 0;
    if (!cyw_arm_regs ||
        !bp_read32(cyw_arm_regs + ARMCR4_CAP, &cap))
        return 0U;
    u32 banks = (cap & ARMCR4_TCBANB_MASK) +
                ((cap & ARMCR4_TCBBNB_MASK) >> ARMCR4_TCBBNB_SHIFT);
    u32 bytes = 0;
    uart_puts("[cyw] CR4 CAP=");
    uart_hex(cap);
    uart_puts(" banks=");
    uart_hex(banks);
    uart_puts("\n");
    for (u32 idx = 0; idx < banks; idx++) {
        u32 info = 0;
        if (!bp_write32(cyw_arm_regs + ARMCR4_BANKIDX, idx) ||
            !bp_read32(cyw_arm_regs + ARMCR4_BANKINFO, &info))
            return 0U;
        u32 block = (info & ARMCR4_BLK_1K_MASK) ?
                    (ARMCR4_BSZ_MULT >> 3) : ARMCR4_BSZ_MULT;
        bytes += ((info & ARMCR4_BSZ_MASK) + 1U) * block;
        uart_puts("[cyw] CR4 bank=");
        uart_hex(idx);
        uart_puts(" info=");
        uart_hex(info);
        uart_puts(" total=");
        uart_hex(bytes);
        uart_puts("\n");
    }
    return bytes;
}

/* ── Chip identification ── */

static bool chip_identify(void)
{
    u32 chip_id;
    if (!bp_read32(CYW_CHIPCOMMON_BASE, &chip_id))
        return false;

    u16 id = (u16)(chip_id & 0xFFFF);
    u16 rev = (u16)((chip_id >> 16) & 0xF);
    cyw_chip_id = id;

    uart_puts("[cyw] chip=");
    uart_hex(id);
    uart_puts(" r=");
    uart_hex(rev);
    uart_puts("\n");

    if (id != CYW43455_CHIP_ID && id != CYW43430_CHIP_ID) {
        uart_puts("[cyw] bad chip ID\n");
        return false;
    }
    return true;
}

/* ── SDPCM / BCDC ── */

static u32 sdpcm_pending_bytes(void);
static bool sdpcm_recv(u8 *channel, u8 *data, u32 *len);
static void handle_event(const u8 *data, u32 len);
static void bcdc_cache_response(const u8 *frame, u32 len);
static bool bcdc_set_iovar(const char *name, const void *data, u32 data_len,
                           bool wait_response);
static u32 load_le32(const u8 *p);
static void store_le32(u8 *p, u32 value);
static bool scan_store_bss(const u8 *bss, u32 record_len);

struct wpa_sha1 {
    u32 h[5];
    u64 bytes;
    u8 block[64];
    u32 used;
};

static u32 wpa_rol32(u32 v, u32 n) { return (v << n) | (v >> (32U - n)); }

static void wpa_sha1_block(struct wpa_sha1 *s, const u8 *p)
{
    u32 w[80];
    for (u32 i = 0U; i < 16U; i++)
        w[i] = ((u32)p[i * 4U] << 24) | ((u32)p[i * 4U + 1U] << 16) |
               ((u32)p[i * 4U + 2U] << 8) | p[i * 4U + 3U];
    for (u32 i = 16U; i < 80U; i++)
        w[i] = wpa_rol32(w[i - 3U] ^ w[i - 8U] ^ w[i - 14U] ^ w[i - 16U], 1U);
    u32 a=s->h[0], b=s->h[1], c=s->h[2], d=s->h[3], e=s->h[4];
    for (u32 i = 0U; i < 80U; i++) {
        u32 f, k;
        if (i < 20U) { f = (b & c) | (~b & d); k = 0x5A827999U; }
        else if (i < 40U) { f = b ^ c ^ d; k = 0x6ED9EBA1U; }
        else if (i < 60U) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCU; }
        else { f = b ^ c ^ d; k = 0xCA62C1D6U; }
        u32 t = wpa_rol32(a, 5U) + f + e + k + w[i];
        e=d; d=c; c=wpa_rol32(b,30U); b=a; a=t;
    }
    s->h[0]+=a; s->h[1]+=b; s->h[2]+=c; s->h[3]+=d; s->h[4]+=e;
}

static void wpa_sha1_init(struct wpa_sha1 *s)
{
    s->h[0]=0x67452301U; s->h[1]=0xEFCDAB89U; s->h[2]=0x98BADCFEU;
    s->h[3]=0x10325476U; s->h[4]=0xC3D2E1F0U;
    s->bytes=0U; s->used=0U;
}

static void wpa_sha1_update(struct wpa_sha1 *s, const u8 *p, u32 len)
{
    s->bytes += len;
    while (len) {
        u32 n = 64U - s->used;
        if (n > len) n = len;
        memcpy(s->block + s->used, p, n);
        s->used += n; p += n; len -= n;
        if (s->used == 64U) { wpa_sha1_block(s, s->block); s->used = 0U; }
    }
}

static void wpa_sha1_final(struct wpa_sha1 *s, u8 out[20])
{
    u64 bits = s->bytes * 8ULL;
    s->block[s->used++] = 0x80U;
    if (s->used > 56U) {
        while (s->used < 64U) s->block[s->used++] = 0U;
        wpa_sha1_block(s, s->block); s->used = 0U;
    }
    while (s->used < 56U) s->block[s->used++] = 0U;
    for (u32 i = 0U; i < 8U; i++)
        s->block[56U + i] = (u8)(bits >> (56U - i * 8U));
    wpa_sha1_block(s, s->block);
    for (u32 i = 0U; i < 5U; i++) {
        out[i*4U]=(u8)(s->h[i]>>24); out[i*4U+1U]=(u8)(s->h[i]>>16);
        out[i*4U+2U]=(u8)(s->h[i]>>8); out[i*4U+3U]=(u8)s->h[i];
    }
}

static void wpa_hmac_sha1(const u8 *key, u32 key_len,
                          const u8 *data, u32 data_len, u8 out[20])
{
    u8 k[64], inner[20];
    memset(k, 0, sizeof(k));
    if (key_len > 64U) {
        struct wpa_sha1 sh; wpa_sha1_init(&sh);
        wpa_sha1_update(&sh, key, key_len); wpa_sha1_final(&sh, k);
    } else memcpy(k, key, key_len);
    for (u32 i=0U;i<64U;i++) k[i]^=0x36U;
    struct wpa_sha1 sh; wpa_sha1_init(&sh);
    wpa_sha1_update(&sh,k,64U); wpa_sha1_update(&sh,data,data_len);
    wpa_sha1_final(&sh,inner);
    for (u32 i=0U;i<64U;i++) k[i]^=(0x36U^0x5CU);
    wpa_sha1_init(&sh); wpa_sha1_update(&sh,k,64U);
    wpa_sha1_update(&sh,inner,20U); wpa_sha1_final(&sh,out);
    memset(k,0,sizeof(k)); memset(inner,0,sizeof(inner));
}

static int wpa_bytes_cmp(const u8 *a, const u8 *b, u32 n)
{
    for (u32 i=0U;i<n;i++) if (a[i]!=b[i]) return a[i]<b[i]?-1:1;
    return 0;
}

static void wpa_derive_ptk_for(const u8 authenticator[CYW_MAC_LEN],
                               u8 out_ptk[64])
{
    static const u8 label[] = "Pairwise key expansion";
    u8 data[76], input[100], mac[20];
    const u8 *mac1 = cyw_mac, *mac2 = authenticator;
    if (wpa_bytes_cmp(mac1,mac2,6U)>0) { const u8 *t=mac1;mac1=mac2;mac2=t; }
    memcpy(data,mac1,6U); memcpy(data+6U,mac2,6U);
    const u8 *n1=wpa_host.snonce,*n2=wpa_host.anonce;
    if (wpa_bytes_cmp(n1,n2,32U)>0) { const u8 *t=n1;n1=n2;n2=t; }
    memcpy(data+12U,n1,32U); memcpy(data+44U,n2,32U);
    u32 off=0U; memcpy(input+off,label,sizeof(label)-1U);off+=sizeof(label)-1U;
    input[off++]=0U; memcpy(input+off,data,sizeof(data));off+=sizeof(data);
    for (u32 i=0U;i<4U;i++) {
        input[off]= (u8)i;
        wpa_hmac_sha1(wpa_host.pmk,32U,input,off+1U,mac);
        u32 n = (64U-i*20U)>20U?20U:(64U-i*20U);
        memcpy(out_ptk+i*20U,mac,n);
    }
    memset(data,0,sizeof(data)); memset(input,0,sizeof(input)); memset(mac,0,sizeof(mac));
}

static bool wpa_send_m2(const u8 *m1, u32 frame_len)
{
    if (frame_len < 113U || wpa_host.rsn_ie_len == 0U)
        return false;
    const u32 eapol = 14U, key = 18U;
    u32 body_len = 95U + wpa_host.rsn_ie_len;
    u32 total = 14U + 4U + body_len;
    if (total > CYW_EAPOL_MAX)
        return false;
    static u8 frame[CYW_EAPOL_MAX] ALIGNED(64);
    memset(frame,0,total);
    memcpy(frame,m1+6U,6U); memcpy(frame+6U,cyw_mac,6U);
    frame[12]=0x88U; frame[13]=0x8EU;
    frame[eapol]=m1[eapol]; frame[eapol+1U]=3U;
    frame[eapol+2U]=(u8)(body_len>>8); frame[eapol+3U]=(u8)body_len;
    frame[key]=2U; frame[key+1U]=0x01U; frame[key+2U]=0x0AU;
    /* RSN msg 2/4 carries key_length 0 (wpa_supplicant
     * wpa_supplicant_send_2_of_4); echoing M1's length is a WPA-only
     * behaviour and some APs silently drop the frame. */
    frame[key+3U]=0U; frame[key+4U]=0U;
    memcpy(frame+key+5U,m1+key+5U,8U);
    memcpy(frame+key+13U,wpa_host.snonce,32U);
    frame[key+93U]=0U; frame[key+94U]=wpa_host.rsn_ie_len;
    memcpy(frame+key+95U,wpa_host.rsn_ie,wpa_host.rsn_ie_len);
    u8 mic[20];
    wpa_hmac_sha1(wpa_host.ptk,16U,frame+eapol,4U+body_len,mic);
    memcpy(frame+key+77U,mic,16U);
    memset(mic,0,sizeof(mic));
    memcpy(cyw_m2_frame,frame,total);
    cyw_m2_len=total;
    return cyw43_send_frame(frame,total);
}

static bool wpa_mic_valid(const u8 *frame, u32 frame_len)
{
    if (frame_len < 113U)
        return false;
    u32 body_len=((u32)frame[16]<<8)|frame[17];
    if (body_len+18U>frame_len || body_len<95U)
        return false;
    u8 copy[CYW_EAPOL_MAX], mac[20], wire[16];
    u32 eapol_len=4U+body_len;
    if (eapol_len>sizeof(copy))
        return false;
    memcpy(copy,frame+14U,eapol_len);
    memcpy(wire,copy+81U,16U);
    memset(copy+81U,0,16U);
    wpa_hmac_sha1(wpa_host.ptk,16U,copy,eapol_len,mac);
    u8 diff=0U;
    for(u32 i=0U;i<16U;i++) diff|=wire[i]^mac[i];
    memset(copy,0,sizeof(copy));memset(mac,0,sizeof(mac));memset(wire,0,sizeof(wire));
    return diff==0U;
}

static bool wpa_aes_unwrap(const u8 *wrapped,u32 wrapped_len,u8 *plain,u32 *plain_len)
{
    if (!wrapped || !plain || !plain_len || wrapped_len<16U ||
        (wrapped_len&7U)!=0U)
        return false;
    u32 n=wrapped_len/8U-1U;
    if (*plain_len<n*8U)
        return false;
    u8 a[8],block[16],out[16];
    memcpy(a,wrapped,8U);memcpy(plain,wrapped+8U,n*8U);
    struct aes_key aes;
    aes_key_expand(&aes,wpa_host.ptk+16U,128U);
    for(i32 j=5;j>=0;j--) {
        for(i32 i=(i32)n;i>=1;i--) {
            u64 t=(u64)n*(u64)j+(u64)i;
            memcpy(block,a,8U);
            for(u32 k=0U;k<8U;k++)
                block[7U-k]^=(u8)(t>>(k*8U));
            memcpy(block+8U,plain+(u32)(i-1)*8U,8U);
            aes_decrypt_block(&aes,block,out);
            memcpy(a,out,8U);
            memcpy(plain+(u32)(i-1)*8U,out+8U,8U);
        }
    }
    static const u8 iv[8]={0xA6U,0xA6U,0xA6U,0xA6U,0xA6U,0xA6U,0xA6U,0xA6U};
    bool ok=memcmp(a,iv,8U)==0;
    *plain_len=ok?n*8U:0U;
    memset(&aes,0,sizeof(aes));memset(a,0,sizeof(a));memset(block,0,sizeof(block));memset(out,0,sizeof(out));
    return ok;
}

static bool wpa_install_key(u32 index,const u8 *key,u32 key_len,
                            bool pairwise,const u8 *peer,u64 rsc)
{
    if (!key || key_len==0U || key_len>32U)
        return false;
    u8 params[164],*p=params;
    memset(params,0,sizeof(params));
#define WPA_PUT32(v) do { u32 _v=(v);p[0]=(u8)_v;p[1]=(u8)(_v>>8);p[2]=(u8)(_v>>16);p[3]=(u8)(_v>>24);p+=4; } while(0)
#define WPA_PUT16(v) do { u16 _v=(v);p[0]=(u8)_v;p[1]=(u8)(_v>>8);p+=2; } while(0)
    WPA_PUT32(pairwise?0U:index);
    WPA_PUT32(key_len);
    memcpy(p,key,key_len);p+=32U+72U;
    WPA_PUT32(4U);                 /* CRYPTO_ALGO_AES_CCM */
    WPA_PUT32(pairwise?0U:2U);     /* PRIMARY_KEY for GTK */
    p+=12U;
    WPA_PUT32(0U);                 /* IV not initialized by host */
    WPA_PUT32((u32)(rsc>>16));
    WPA_PUT16((u16)rsc);
    p+=10U;
    if(pairwise&&peer) memcpy(p,peer,CYW_MAC_LEN);
#undef WPA_PUT32
#undef WPA_PUT16
    bool ok=bcdc_set_iovar("wsec_key",params,sizeof(params),false);
    memset(params,0,sizeof(params));
    return ok;
}

static bool wpa_find_gtk(const u8 *data,u32 len,u8 *gtk,u32 *gtk_len,u32 *key_id)
{
    u32 off=0U;
    while(off+2U<=len) {
        u32 item=data[off+1U]+2U;
        if(item<2U||item>len-off) return false;
        if(data[off]==0xDDU&&item>=24U&&
           data[off+2U]==0x00U&&data[off+3U]==0x0FU&&
           data[off+4U]==0xACU&&data[off+5U]==0x01U) {
            u32 n=item-8U;
            if(n>32U||*gtk_len<n) return false;
            *key_id=data[off+6U]&3U;
            memcpy(gtk,data+off+8U,n);*gtk_len=n;
            return true;
        }
        off+=item;
    }
    return false;
}

static bool wpa_send_m4(const u8 *m3,u32 frame_len)
{
    if(frame_len<113U) return false;
    static u8 frame[113] ALIGNED(64);
    memset(frame,0,sizeof(frame));
    /* Reply to the authenticator: destination is M3's source, source is us.
     * Copying M3's first 14 bytes verbatim (the previous behaviour) addressed
     * M4 to ourselves with the AP as source, so the AP never saw it and the
     * handshake died after M3. */
    memcpy(frame,m3+6U,6U);
    memcpy(frame+6U,cyw_mac,6U);
    frame[12]=0x88U;frame[13]=0x8EU;
    frame[14]=m3[14];frame[15]=3U;frame[16]=0U;frame[17]=95U;
    frame[18]=2U;frame[19]=0x03U;frame[20]=0x0AU;
    /* RSN msg 4/4 carries key_length 0, as msg 2/4 does. */
    frame[21]=0U;frame[22]=0U;
    memcpy(frame+23U,m3+23U,8U);
    u8 mac[20];
    wpa_hmac_sha1(wpa_host.ptk,16U,frame+14U,99U,mac);
    memcpy(frame+95U,mac,16U);memset(mac,0,sizeof(mac));
    return cyw43_send_frame(frame,sizeof(frame));
}

static bool wpa_handle_m3(const u8 *frame,u32 frame_len)
{
    const u32 key=18U;
    if(frame_len<113U||!wpa_mic_valid(frame,frame_len))
        return false;
    if(memcmp(frame+key+13U,wpa_host.anonce,32U)!=0)
        return false;
    u64 replay=((u64)frame[23]<<56)|((u64)frame[24]<<48)|
               ((u64)frame[25]<<40)|((u64)frame[26]<<32)|
               ((u64)frame[27]<<24)|((u64)frame[28]<<16)|
               ((u64)frame[29]<<8)|frame[30];
    if(replay<wpa_host.replay)
        return false;
    u32 kd_len=((u32)frame[key+93U]<<8)|frame[key+94U];
    if(key+95U+kd_len>frame_len)
        return false;
    u8 plain[256],gtk[32];u32 plain_len=sizeof(plain),gtk_len=sizeof(gtk),key_id=0U;
    u16 info=((u16)frame[19]<<8)|frame[20];
    const u8 *kd=frame+key+95U;
    if(info&0x1000U) {
        if(kd_len>sizeof(plain)||!wpa_aes_unwrap(kd,kd_len,plain,&plain_len))
            return false;
        kd=plain;kd_len=plain_len;
    }
    if(!wpa_find_gtk(kd,kd_len,gtk,&gtk_len,&key_id))
        return false;
    u64 rsc=0ULL;
    for(u32 i=0U;i<8U;i++) rsc=(rsc<<8)|frame[key+61U+i];
    /* Send msg 4/4 before installing keys, as wpa_supplicant does: once the
     * pairwise key is installed the firmware encrypts outbound frames, and the
     * authenticator has not yet keyed the link for our M4. */
    bool ok=wpa_send_m4(frame,frame_len)&&
            wpa_install_key(0U,wpa_host.ptk+32U,16U,true,wpa_host.ap_mac,0ULL)&&
            wpa_install_key(key_id,gtk,gtk_len,false,NULL,rsc);
    memset(plain,0,sizeof(plain));memset(gtk,0,sizeof(gtk));
    if(ok){wpa_host.keys_installed=true;cyw_link=CYW_LINK_UP;}
    return ok;
}

static void capture_eapol(const u8 *data, u32 len)
{
    if (!data || len < 4U)
        return;
    u32 bdc_offset = 4U + ((u32)data[3] << 2);
    if (bdc_offset > len || len - bdc_offset < 14U)
        return;
    const u8 *frame = data + bdc_offset;
    u32 frame_len = len - bdc_offset;
    if (frame[12] != 0x88U || frame[13] != 0x8EU)
        return;

    u32 copy_len = frame_len;
    if (copy_len > sizeof(cyw_eapol_frame))
        copy_len = sizeof(cyw_eapol_frame);
    memcpy(cyw_eapol_frame, frame, copy_len);
    cyw_eapol_len = copy_len;
    cyw_diag.eapol_frames++;
    cyw_diag.eapol_len = frame_len;
    cyw_diag.eapol_key_info =
        frame_len >= 21U ? ((u32)frame[19] << 8) | frame[20] : 0U;
    cyw_diag.eapol_replay_hi =
        frame_len >= 31U ? ((u32)frame[23] << 24) |
                           ((u32)frame[24] << 16) |
                           ((u32)frame[25] << 8) | frame[26] : 0U;
    cyw_diag.eapol_replay_lo =
        frame_len >= 31U ? ((u32)frame[27] << 24) |
                           ((u32)frame[28] << 16) |
                           ((u32)frame[29] << 8) | frame[30] : 0U;
    memset(cyw_diag.eapol_words, 0, sizeof(cyw_diag.eapol_words));
    u32 words = frame_len / 4U;
    if (words > 9U)
        words = 9U;
    for (u32 i = 0U; i < words; i++)
        cyw_diag.eapol_words[i] = load_le32(frame + i * 4U);
    uart_puts("[cyw] EAPOL keyinfo=");
    uart_hex(cyw_diag.eapol_key_info);
    uart_puts(" replay=");
    uart_hex(cyw_diag.eapol_replay_hi);
    uart_puts(":");
    uart_hex(cyw_diag.eapol_replay_lo);
    uart_puts("\n");

    if (wpa_host.enabled && frame_len >= 113U) {
        u16 key_info = ((u16)frame[19] << 8) | frame[20];
        bool pairwise = (key_info & 0x0008U) != 0U;
        bool ack = (key_info & 0x0080U) != 0U;
        bool mic = (key_info & 0x0100U) != 0U;
        if (pairwise && ack && !mic) {
            memcpy(wpa_host.anonce,frame+31U,32U);
            wpa_host.replay =
                ((u64)frame[23]<<56)|((u64)frame[24]<<48)|
                ((u64)frame[25]<<40)|((u64)frame[26]<<32)|
                ((u64)frame[27]<<24)|((u64)frame[28]<<16)|
                ((u64)frame[29]<<8)|frame[30];
            if (!wpa_host.nonce_ready) {
                u8 seed[28];
                memcpy(seed,cyw_mac,6U);
                memcpy(seed+6U,wpa_host.ap_mac,6U);
                u64 now=timer_monotonic_ms();
                for (u32 i=0U;i<8U;i++)
                    seed[12U+i]=(u8)(now>>(i*8U));
                u64 ticks=timer_ticks();
                for (u32 i=0U;i<8U;i++)
                    seed[20U+i]=(u8)(ticks>>(i*8U));
                hmac_sha256(wpa_host.pmk,32U,seed,sizeof(seed),
                            wpa_host.snonce);
                memset(seed,0,sizeof(seed));
                wpa_host.nonce_ready=true;
            }

            /* The association target can be a transmitted/parent BSSID in a
             * multi-BSSID profile. The EAPOL source is the authenticator
             * address used by the four-way handshake. */
            memcpy(wpa_host.ap_mac,frame+6U,CYW_MAC_LEN);
            wpa_derive_ptk_for(wpa_host.ap_mac,wpa_host.ptk);
            if (wpa_send_m2(frame,frame_len)) {
                wpa_host.m2_sent = true;
                cyw_diag.eapol_m2_sent++;
            } else {
                cyw_diag.eapol_m2_fail++;
            }
        } else if (pairwise && ack && mic)
            (void)wpa_handle_m3(frame,frame_len);
    }
}

static bool cyw_data_is_eapol(const u8 *data, u32 len)
{
    if (!data || len < 4U)
        return false;
    u32 bdc_offset = 4U + ((u32)data[3] << 2);
    return bdc_offset <= len && len - bdc_offset >= 14U &&
           data[bdc_offset + 12U] == 0x88U &&
           data[bdc_offset + 13U] == 0x8EU;
}

static bool cyw_data_queue_push(const u8 *data, u32 len)
{
    if (!data || len > CYW_MAX_FRAME ||
        cyw_data_rx_count == CYW_DATA_RX_QUEUE_DEPTH) {
        cyw_diag.data_rx_dropped++;
        return false;
    }
    struct cyw_data_rx_slot *slot = &cyw_data_rx_queue[cyw_data_rx_head];
    memcpy(slot->data, data, len);
    slot->len = len;
    cyw_data_rx_head =
        (cyw_data_rx_head + 1U) % CYW_DATA_RX_QUEUE_DEPTH;
    cyw_data_rx_count++;
    cyw_diag.data_rx_queued++;
    return true;
}

static bool cyw_data_frame_copy(const u8 *data, u32 data_len,
                                u8 *frame, u32 *frame_len)
{
    if (!data || !frame || !frame_len || data_len < 4U)
        return false;
    u32 bdc_offset = 4U + ((u32)data[3] << 2);
    if (bdc_offset >= data_len)
        return false;
    u32 payload_len = data_len - bdc_offset;
    if (payload_len > *frame_len)
        payload_len = *frame_len;
    memcpy(frame, data + bdc_offset, payload_len);
    *frame_len = payload_len;
    return true;
}

#define CYW_SCAN_SEC_WPA2_PSK        (1U << 0)
#define CYW_SCAN_SEC_PSK_SHA256      (1U << 1)
#define CYW_SCAN_SEC_SAE             (1U << 2)
#define CYW_SCAN_SEC_MFP_CAPABLE     (1U << 3)
#define CYW_SCAN_SEC_MFP_REQUIRED    (1U << 4)

static u32 sdpcm_build_header(u8 *buf, u32 payload_len, u8 channel)
{
    u32 total = SDPCM_HEADER_LEN + payload_len;

    /* Frame tag: length + ~length */
    buf[0] = (u8)(total & 0xFF);
    buf[1] = (u8)((total >> 8) & 0xFF);
    buf[2] = (u8)(~total & 0xFF);
    buf[3] = (u8)((~total >> 8) & 0xFF);

    /* Sequence, channel, next length, data offset */
    buf[4] = cyw_tx_seq++;
    buf[5] = channel;
    buf[6] = 0;    /* next length */
    buf[7] = SDPCM_HEADER_LEN;  /* data offset */

    /* Flow control, max seq */
    buf[8] = 0;
    buf[9] = 0;
    buf[10] = 0;
    buf[11] = 0;

    return SDPCM_HEADER_LEN;
}

static bool sdpcm_can_send(u8 channel)
{
    if (channel >= 8U)
        return false;
    if ((u8)(cyw_tx_max - cyw_tx_seq) == 0U)
        return false;
    return channel != SDPCM_DATA_CHANNEL ||
           (cyw_tx_fcmask & (u8)(1U << SDPCM_DATA_CHANNEL)) == 0U;
}

static bool sdpcm_send(u8 channel, const u8 *data, u32 len)
{
    if (channel >= 8U)
        return false;
    if (len + SDPCM_HEADER_LEN > CYW_MAX_FRAME)
        return false;

    /* Firmware owns the transmit window. Do not publish another frame until
     * max_seq advances; drain queued responses/events to acquire credit.
     *
     * This loop deliberately does NOT pet the hardware watchdog, so a stall
     * cannot hang the board indefinitely. It must therefore give up well
     * before the ~15 s hardware watchdog deadline: letting the watchdog win
     * resets the board with no record of why, which is exactly the failure
     * mode that made the CYW43455 RX regression so expensive to diagnose.
     * On expiry we PiSOD, persist the record to SD, and reboot deliberately.
     * Callers that must not block (reactor/association pokes) check credit
     * before calling. */
    u64 stall_start = timer_monotonic_ms();
    u64 credit_deadline = stall_start + CYW_TX_CREDIT_STALL_MS;
    while ((u8)(cyw_tx_max - cyw_tx_seq) == 0U ||
           (channel == SDPCM_DATA_CHANNEL &&
            (cyw_tx_fcmask & (u8)(1U << SDPCM_DATA_CHANNEL)) != 0U)) {
        if (sdpcm_pending_bytes() != 0U) {
            u8 pending_channel = 0;
            u32 pending_len = CYW_MAX_FRAME;
            if (sdpcm_recv(&pending_channel, cyw_rx_buf, &pending_len)) {
                if (pending_channel == SDPCM_EVENT_CHANNEL)
                    handle_event(cyw_rx_buf, pending_len);
                else if (pending_channel == SDPCM_CTL_CHANNEL)
                    bcdc_cache_response(cyw_rx_buf, pending_len);
            }
        } else {
            timer_delay_ms(1U);
        }
        if (timer_monotonic_ms() >= credit_deadline) {
            u64 values[8] = {
                (u8)(cyw_tx_max - cyw_tx_seq),
                cyw_tx_seq,
                cyw_tx_max,
                cyw_tx_fcmask,
                channel,
                cyw_diag.stage,
                cyw_diag.bcdc_rframe_count,
                cyw_diag.event_count,
            };
            exception_pisod_reboot("CYW43455 SDPCM TX credit stall",
                                   EXCEPTION_CRASH_KIND_STALL,
                                   EXCEPTION_STALL_CYW_TX_CREDIT,
                                   values, 8U);
        }
    }

    u32 hdr_len = sdpcm_build_header(cyw_tx_buf, len, channel);
    memcpy(cyw_tx_buf + hdr_len, data, len);

    u32 total = hdr_len + len;
    /* The SDPCM FIFO requires word alignment for short transfers, not a full
     * function-1-style 64-byte block. Writing 192 bytes for a declared
     * 151-byte EAPOL frame left trailing bytes in the FIFO; Circle and
     * brcmfmac round short function-2 transfers to 4 bytes. */
    u32 padded = (total + 3U) & ~3U;

    /* Use full 512-byte blocks only when the frame actually exceeds one. */
    if (padded > 512) {
        u32 nblks = (padded + SDIO_FUNC2_BLKSZ - 1) / SDIO_FUNC2_BLKSZ;
        u32 transfer = nblks * SDIO_FUNC2_BLKSZ;
        if (transfer > total)
            memset(cyw_tx_buf + total, 0, transfer - total);
        return sdio_cmd53_write_blocks(SDIO_FUNC_WLAN, 0, cyw_tx_buf,
                                       SDIO_FUNC2_BLKSZ, nblks, false);
    }
    if (padded > total)
        memset(cyw_tx_buf + total, 0, padded - total);
    return sdio_cmd53_write(SDIO_FUNC_WLAN, 0, cyw_tx_buf, padded, false);
}

static u32 sdpcm_pending_bytes(void)
{
    /* A chained next-length is free information carried in the previous frame's
     * header. Act on it without touching the bus, and never throttle it: it is
     * how back-to-back frames are drained at full rate. */
    if (sdpcm_next_len != 0U) {
        cyw_diag.bcdc_pending_bytes = sdpcm_next_len;
        return sdpcm_next_len;
    }

    /* Everything below costs real SDIO transactions, and cyw43_poll() runs on
     * core 0 on every reactor pass. Discovering "nothing to do" was costing
     * ~63% of core 0 whenever WiFi was initialised, connected or not, because
     * each pass issued the RFRAMEBC pair plus a windowed backplane read.
     *
     * The card's in-band interrupt (SDHCI status bit 8) answers "is there
     * anything at all?" in one MMIO read, with no bus transaction. Treat it as
     * an accelerator only: an asserted line collapses the backoff so latency
     * stays interrupt-like, but a quiet line still falls through to the paced
     * probe. That ordering matters -- gating RX on the card interrupt outright
     * would resurrect the original dead-RX bug on any board where the line does
     * not behave. */
    if (cyw_rx_irq_hint) {
        cyw_rx_irq_hint = false;
        cyw_diag.rx_card_irqs++;
        sdio_card_irq_ack();
        cyw_rx_probe_backoff_ms = 0U;
        cyw_rx_probe_next_ms = 0U;
    }
    bool probe_due = cyw_rx_probe_armed ||
                     timer_monotonic_ms() >= cyw_rx_probe_next_ms;
    if (!probe_due) {
        cyw_diag.bcdc_pending_bytes = 0U;
        return 0U;
    }

    u8 lo = 0, hi = 0;
    if (!sdio_cmd52_read(SDIO_FUNC_BACKPLANE, SDIO_RFRAMEBC_LO, &lo) ||
        !sdio_cmd52_read(SDIO_FUNC_BACKPLANE, SDIO_RFRAMEBC_HI, &hi))
        return 0U;
    u32 count = (u32)lo | ((u32)hi << 8);
    cyw_diag.bcdc_rframe_count = count;
    if (count == 0U) {
        u32 intstatus = 0;
        if (bp_read32(cyw_sdio_regs + SDIOD_INTSTATUS, &intstatus)) {
            u32 active = intstatus & 0x200000F0U;
            if (active)
                (void)bp_write32(cyw_sdio_regs + SDIOD_INTSTATUS, active);
            if (active & SDIOD_I_HMB_HOST_INT) {
                u32 mailbox = 0;
                if (bp_read32(cyw_sdio_regs + SDIOD_TOHOSTMAILBOXDATA,
                              &mailbox)) {
                    cyw_diag.sdiod_mailbox = mailbox;
                    (void)bp_write32(cyw_sdio_regs + SDIOD_TOSBMAILBOX,
                                     SDIOD_SMB_INT_ACK);
                }
            }
            if (active & SDIOD_I_HMB_FRAME_IND)
                sdpcm_frame_pending = true;
            cyw_diag.sdiod_intstatus = intstatus;
        }
        u8 pending = 0;
        if (sdio_cmd52_read(SDIO_FUNC_CIA, CCCR_INT_PENDING, &pending) &&
            (pending & (1U << SDIO_FUNC_WLAN)))
            sdpcm_frame_pending = true;
        cyw_diag.bcdc_cccr_pending = pending;
    }
    if (count == 0U && sdpcm_next_len != 0U)
        count = sdpcm_next_len;
    else if (count == 0U && sdpcm_frame_pending)
        count = 64U; /* header-first read discovers the exact frame length */
    else if (count == 0U && cyw_rx_probe_enabled) {
        /* Speculative header probe.
         *
         * RFRAMEBC, the chained next-length field and the mailbox/CCCR frame
         * indications are all indications we may simply never see: on this
         * board RFRAMEBC reads 0 permanently and no HMB frame indication ever
         * fires, so once a next-length chain ends the driver has no trigger
         * left and RX stops for good (events=0, scan never completes, and TX
         * credit -- which is only refreshed from a received SDPCM header --
         * freezes until the send path stalls).
         *
         * brcmf_sdio_readframes() does not wait to be told either: it reads a
         * header and decides from its contents whether a frame was really
         * there. Reading an empty function-2 FIFO yields zeros, which the
         * length/~length tag check in sdpcm_recv() rejects.
         *
         * The decision is latched in cyw_rx_probe_armed because callers ask
         * twice per frame (cyw43_poll() to test, sdpcm_recv() to size the
         * read); without the latch the second call would fall on the wrong
         * side of the backoff deadline and abort a probe we just authorised. */
        if (!cyw_rx_probe_armed) {
            cyw_rx_probe_armed = true;
            cyw_diag.rx_probe_attempts++;
        }
        count = 64U;
    }
    cyw_diag.bcdc_pending_bytes = count;
    return count;
}

/* Outcome of a speculative probe. A frame means more are probably queued, so
 * poll flat out; an empty FIFO means back off geometrically to a ceiling. This
 * keeps a quiet link near zero cost while preserving full throughput under
 * traffic -- an unthrottled probe pinned core 0 at ~90%. */
static void sdpcm_probe_result(bool got_frame)
{
    if (got_frame) {
        /* Frames arrive in bursts; go flat out until the FIFO runs dry. */
        cyw_rx_probe_armed = false;
        cyw_rx_probe_backoff_ms = 0U;
        cyw_rx_probe_next_ms = 0U;
        return;
    }
    if (!cyw_rx_probe_armed)
        return;
    cyw_rx_probe_armed = false;
    cyw_rx_probe_backoff_ms = cyw_rx_probe_backoff_ms == 0U
        ? 1U
        : (cyw_rx_probe_backoff_ms << 1);
    if (cyw_rx_probe_backoff_ms > CYW_RX_PROBE_BACKOFF_MAX_MS)
        cyw_rx_probe_backoff_ms = CYW_RX_PROBE_BACKOFF_MAX_MS;
    cyw_rx_probe_next_ms = timer_monotonic_ms() + cyw_rx_probe_backoff_ms;
}

static bool sdpcm_recv(u8 *channel, u8 *data, u32 *len)
{
    u32 pending = sdpcm_pending_bytes();
    if (pending < SDPCM_HEADER_LEN || pending > CYW_MAX_FRAME) {
        sdpcm_probe_result(false);
        return false;
    }
    u32 reported = cyw_diag.bcdc_rframe_count;
    if (reported == 0U && sdpcm_next_len != 0U)
        reported = sdpcm_next_len;

    /* A frame indication authorizes one header-first FIFO probe. If that probe
     * fails, require a fresh RFRAMEBC/interrupt indication before retrying. */
    sdpcm_frame_pending = false;
    u32 first = pending < 64U ? ((pending + 3U) & ~3U) : 64U;
    if (!sdio_cmd53_read(SDIO_FUNC_WLAN, 0, cyw_rx_buf, first, false)) {
        sdpcm_probe_result(false);
        return false;
    }
    sdpcm_next_len = 0U;

    /* Parse frame tag */
    u16 frame_len = (u16)cyw_rx_buf[0] | ((u16)cyw_rx_buf[1] << 8);
    u16 frame_not = (u16)cyw_rx_buf[2] | ((u16)cyw_rx_buf[3] << 8);

    /* An empty function-2 FIFO reads back as zeroes, so this is also how a
     * speculative probe learns that there was nothing to collect. */
    if (frame_len == 0 || frame_len > CYW_MAX_FRAME) {
        sdpcm_probe_result(false);
        return false;
    }
    if ((u16)(frame_len ^ frame_not) != 0xFFFF) {
        sdpcm_probe_result(false);
        return false;
    }
    sdpcm_probe_result(true);
    u32 rounded = (frame_len + 3U) & ~3U;
    u32 transfer_len = rounded;
    if (reported >= frame_len && reported <= CYW_MAX_FRAME)
        transfer_len = (reported + 3U) & ~3U;
    u32 have = first;
    while (have < transfer_len) {
        u32 chunk = transfer_len - have;
        if (chunk > 512U) chunk = 512U;
        if (!sdio_cmd53_read(SDIO_FUNC_WLAN, 0,
                             cyw_rx_buf + have, chunk, false))
            return false;
        have += chunk;
    }

    *channel = cyw_rx_buf[5] & 0x0F;
    sdpcm_next_len = (u32)cyw_rx_buf[6] << 4;
    cyw_tx_fcmask = cyw_rx_buf[8];
    cyw_tx_max = cyw_rx_buf[9];
    cyw_diag.rx_frames++;
    cyw_diag.bcdc_last_channel = *channel;
    cyw_diag.bcdc_last_frame_len = frame_len;
    u8 doff = cyw_rx_buf[7];
    if (doff < SDPCM_HEADER_LEN || doff > frame_len)
        return false;

    u32 payload_len = frame_len - doff;
    if (payload_len > (u32)(CYW_MAX_FRAME - doff))
        return false;

    memcpy(data, cyw_rx_buf + doff, payload_len);
    *len = payload_len;
    return true;
}

/* ── BCDC control messages ── */

static bool bcdc_get_cmd(u32 cmd, u8 *data, u32 data_len, u32 *resp_len);

static bool bcdc_response_ignored(u16 id)
{
    return bcdc_ignore_through != 0U &&
           (u16)(bcdc_ignore_through - id) < 0x8000U;
}

static void bcdc_cache_response(const u8 *frame, u32 len)
{
    if (!frame || len < BCDC_HEADER_LEN)
        return;
    u16 id = (u16)frame[10] | ((u16)frame[11] << 8);
    u32 status = (u32)frame[12] |
                 ((u32)frame[13] << 8) |
                 ((u32)frame[14] << 16) |
                 ((u32)frame[15] << 24);
    cyw_diag.bcdc_response_id = id;
    cyw_diag.bcdc_status = status;
    if (bcdc_response_ignored(id))
        return;

    u32 slot = BCDC_RESPONSE_SLOTS;
    for (u32 i = 0U; i < BCDC_RESPONSE_SLOTS; i++) {
        if (bcdc_responses[i].valid && bcdc_responses[i].id == id) {
            slot = i;
            break;
        }
        if (!bcdc_responses[i].valid && slot == BCDC_RESPONSE_SLOTS)
            slot = i;
    }
    if (slot == BCDC_RESPONSE_SLOTS)
        slot = id % BCDC_RESPONSE_SLOTS;

    struct bcdc_cached_response *cached = &bcdc_responses[slot];
    cached->id = id;
    cached->status = status;
    u32 data_len = len - BCDC_HEADER_LEN;
    if (data_len > BCDC_RESPONSE_DATA_MAX)
        data_len = BCDC_RESPONSE_DATA_MAX;
    cached->data_len = (u16)data_len;
    if (data_len != 0U)
        memcpy(cached->data, frame + BCDC_HEADER_LEN, data_len);
    cached->valid = true;
}

static bool bcdc_take_response(u16 id, u8 *data, u32 *data_len, u32 *status)
{
    for (u32 i = 0U; i < BCDC_RESPONSE_SLOTS; i++) {
        struct bcdc_cached_response *cached = &bcdc_responses[i];
        if (!cached->valid || cached->id != id)
            continue;
        if (status)
            *status = cached->status;
        if (data_len) {
            u32 copy_len = cached->data_len;
            if (copy_len > *data_len)
                copy_len = *data_len;
            if (data && copy_len != 0U)
                memcpy(data, cached->data, copy_len);
            *data_len = copy_len;
        }
        cached->valid = false;
        return true;
    }
    return false;
}

static bool bcdc_set_iovar(const char *name, const void *data, u32 data_len,
                           bool wait_response)
{
    cyw_diag.bcdc_calls++;
    u32 name_len = pios_strlen(name) + 1;
    u32 body_len = name_len + data_len;
    u32 payload_len = BCDC_HEADER_LEN + body_len;

    if (payload_len > CYW_MAX_FRAME - SDPCM_HEADER_LEN)
        return false;

    static u8 buf[CYW_MAX_FRAME] ALIGNED(64);
    if (payload_len > sizeof(buf))
        return false;
    u16 id = bcdc_reqid++;
    cyw_diag.bcdc_request_id = id;

    /* BCDC 16-byte header (little-endian) */
    u32 cmd = WLC_SET_VAR;
    buf[0]  = (u8)(cmd & 0xFF);
    buf[1]  = (u8)((cmd >> 8) & 0xFF);
    buf[2]  = (u8)((cmd >> 16) & 0xFF);
    buf[3]  = (u8)((cmd >> 24) & 0xFF);
    buf[4]  = (u8)(body_len & 0xFF);
    buf[5]  = (u8)((body_len >> 8) & 0xFF);
    buf[6]  = (u8)((body_len >> 16) & 0xFF);
    buf[7]  = (u8)((body_len >> 24) & 0xFF);
    u16 flags = BCDC_FLAG_SET;
    buf[8]  = (u8)(flags & 0xFF);
    buf[9]  = (u8)((flags >> 8) & 0xFF);
    buf[10] = (u8)(id & 0xFF);
    buf[11] = (u8)((id >> 8) & 0xFF);
    buf[12] = 0; buf[13] = 0; buf[14] = 0; buf[15] = 0; /* status = 0 */

    /* iovar name */
    memcpy(buf + BCDC_HEADER_LEN, name, name_len);

    /* data */
    if (data && data_len > 0)
        memcpy(buf + BCDC_HEADER_LEN + name_len, data, data_len);

    if (!sdpcm_send(SDPCM_CTL_CHANNEL, buf, payload_len)) {
        cyw_diag.bcdc_send_failures++;
        return false;
    }
    if (!wait_response) {
        bcdc_ignore_through = id;
        return true;
    }

    /* Regulatory CLM processing can take several seconds before firmware
     * emits the correlated BCDC response. Keep the wait bounded but long
     * enough that its reply is not misclassified as stale by the next call. */
    u64 deadline = timer_monotonic_ms() + 15000ULL;
    u32 polls = 0U;
    while (timer_monotonic_ms() < deadline) {
        u32 status = 0U;
        if (bcdc_take_response(id, NULL, NULL, &status)) {
            cyw_diag.bcdc_response_id = id;
            cyw_diag.bcdc_status = status;
            return status == 0U;
        }
        if ((polls++ & 127U) == 0U)
            watchdog_hw_pet();
        if (sdpcm_pending_bytes() == 0U) {
            timer_delay_ms(1U);
            continue;
        }
        u8 channel = 0;
        u32 rlen = CYW_MAX_FRAME;
        if (!sdpcm_recv(&channel, cyw_rx_buf, &rlen))
            continue;
        if (channel == SDPCM_EVENT_CHANNEL) {
            handle_event(cyw_rx_buf, rlen);
            continue;
        }
        if (channel != SDPCM_CTL_CHANNEL || rlen < BCDC_HEADER_LEN)
            continue;
        bcdc_cache_response(cyw_rx_buf, rlen);
    }
    cyw_diag.bcdc_timeouts++;
    (void)bp_read32(cyw_sdio_regs + SDIOD_INTSTATUS,
                    &cyw_diag.sdiod_intstatus);
    (void)bp_read32(cyw_sdio_regs + SDIOD_INTMASK,
                    &cyw_diag.sdiod_intmask);
    (void)bp_read32(cyw_sdio_regs + SDIOD_TOHOSTMAILBOXDATA,
                    &cyw_diag.sdiod_mailbox);
    /* WLC_SET_VAR is accepted asynchronously by this firmware. The command
     * response may trail subsequent requests by tens of seconds; scan/join
     * success is authoritatively reported by firmware events instead. */
    return true;
}

static bool bcdc_get_iovar(const char *name, u8 *resp, u32 *resp_len)
{
    if (!name || !resp_len || *resp_len == 0U || *resp_len > 512U)
        return false;
    u32 name_len = pios_strlen(name) + 1;
    if (name_len > *resp_len)
        return false;

    u8 query[512];
    u32 query_len = *resp_len;
    memset(query, 0, query_len);
    memcpy(query, name, name_len);
    u32 actual = query_len;
    if (!bcdc_get_cmd(WLC_GET_VAR, query, query_len, &actual))
        return false;
    if (resp && actual != 0U)
        memcpy(resp, query, actual);
    *resp_len = actual;
    return true;
}

static bool bcdc_set_cmd(u32 cmd, const void *data, u32 data_len,
                         bool wait_response)
{
    u32 payload_len = BCDC_HEADER_LEN + data_len;
    if (payload_len > sizeof(cyw_tx_buf) - SDPCM_HEADER_LEN)
        return false;

    static u8 buf[CYW_MAX_FRAME] ALIGNED(64);
    if (payload_len > sizeof(buf))
        return false;
    u16 id = bcdc_reqid++;

    /* BCDC 16-byte header (little-endian) */
    buf[0]  = (u8)(cmd & 0xFF);
    buf[1]  = (u8)((cmd >> 8) & 0xFF);
    buf[2]  = (u8)((cmd >> 16) & 0xFF);
    buf[3]  = (u8)((cmd >> 24) & 0xFF);
    buf[4]  = (u8)(data_len & 0xFF);
    buf[5]  = (u8)((data_len >> 8) & 0xFF);
    buf[6]  = (u8)((data_len >> 16) & 0xFF);
    buf[7]  = (u8)((data_len >> 24) & 0xFF);
    u16 flags = BCDC_FLAG_SET;
    buf[8]  = (u8)(flags & 0xFF);
    buf[9]  = (u8)((flags >> 8) & 0xFF);
    buf[10] = (u8)(id & 0xFF);
    buf[11] = (u8)((id >> 8) & 0xFF);
    buf[12] = 0; buf[13] = 0; buf[14] = 0; buf[15] = 0; /* status = 0 */

    if (data && data_len > 0)
        memcpy(buf + BCDC_HEADER_LEN, data, data_len);

    if (!sdpcm_send(SDPCM_CTL_CHANNEL, buf, payload_len))
        return false;
    if (wait_response) {
        u64 deadline = timer_monotonic_ms() + 15000ULL;
        u32 polls = 0U;
        while (timer_monotonic_ms() < deadline) {
            u32 status = 0U;
            if (bcdc_take_response(id, NULL, NULL, &status)) {
                cyw_diag.bcdc_response_id = id;
                cyw_diag.bcdc_status = status;
                return status == 0U;
            }
            if ((polls++ & 127U) == 0U)
                watchdog_hw_pet();
            if (sdpcm_pending_bytes() == 0U) {
                timer_delay_ms(1U);
                continue;
            }
            u8 channel = 0U;
            u32 rlen = CYW_MAX_FRAME;
            if (!sdpcm_recv(&channel, cyw_rx_buf, &rlen))
                continue;
            if (channel == SDPCM_EVENT_CHANNEL)
                handle_event(cyw_rx_buf, rlen);
            else if (channel == SDPCM_CTL_CHANNEL)
                bcdc_cache_response(cyw_rx_buf, rlen);
        }
        cyw_diag.bcdc_timeouts++;
        return false;
    }
    bcdc_ignore_through = id;
    return true;
}

static bool bcdc_get_cmd(u32 cmd, u8 *data, u32 data_len, u32 *resp_len)
{
    u32 payload_len = BCDC_HEADER_LEN + data_len;
    if (!resp_len || payload_len > CYW_MAX_FRAME - SDPCM_HEADER_LEN)
        return false;

    static u8 buf[CYW_MAX_FRAME] ALIGNED(64);
    u16 ids[3];
    cyw_diag.bcdc_calls += 3U;

    /* BCDC 16-byte header (little-endian) */
    buf[0]  = (u8)(cmd & 0xFF);
    buf[1]  = (u8)((cmd >> 8) & 0xFF);
    buf[2]  = (u8)((cmd >> 16) & 0xFF);
    buf[3]  = (u8)((cmd >> 24) & 0xFF);
    buf[4]  = (u8)(data_len & 0xFF);
    buf[5]  = (u8)((data_len >> 8) & 0xFF);
    buf[6]  = (u8)((data_len >> 16) & 0xFF);
    buf[7]  = (u8)((data_len >> 24) & 0xFF);
    buf[8]  = 0; buf[9] = 0;   /* flags = 0 (GET) */
    buf[12] = 0; buf[13] = 0; buf[14] = 0; buf[15] = 0; /* status = 0 */

    if (data && data_len > 0U)
        memcpy(buf + BCDC_HEADER_LEN, data, data_len);

    for (u32 probe = 0U; probe < 3U; probe++) {
        ids[probe] = bcdc_reqid++;
        buf[10] = (u8)(ids[probe] & 0xFF);
        buf[11] = (u8)((ids[probe] >> 8) & 0xFF);
        cyw_diag.bcdc_request_id = ids[probe];
        if (!sdpcm_send(SDPCM_CTL_CHANNEL, buf, payload_len)) {
            cyw_diag.bcdc_send_failures++;
            return false;
        }
    }

    u64 deadline = timer_monotonic_ms() + CYW_BCDC_GET_TIMEOUT_MS;
    u32 polls = 0U;
    while (timer_monotonic_ms() < deadline) {
        for (u32 probe = 0U; probe < 3U; probe++) {
            u32 actual = data_len;
            u32 status = 0U;
            if (bcdc_take_response(ids[probe], data, &actual, &status)) {
                cyw_diag.bcdc_response_id = ids[probe];
                cyw_diag.bcdc_status = status;
                *resp_len = actual;
                return status == 0U;
            }
        }
        if ((polls++ & 127U) == 0U)
            watchdog_hw_pet();
        if (sdpcm_pending_bytes() == 0U) {
            timer_delay_ms(1U);
            continue;
        }
        u8 channel = 0U;
        u32 rlen = CYW_MAX_FRAME;
        if (!sdpcm_recv(&channel, cyw_rx_buf, &rlen))
            continue;
        if (channel == SDPCM_EVENT_CHANNEL) {
            handle_event(cyw_rx_buf, rlen);
            continue;
        }
        if (channel != SDPCM_CTL_CHANNEL || rlen < BCDC_HEADER_LEN)
            continue;
        bcdc_cache_response(cyw_rx_buf, rlen);
    }
    cyw_diag.bcdc_timeouts++;
    return false;
}

/* ── Event handling ── */

static void event_history_reset(void)
{
    memset(&cyw_event_history, 0, sizeof(cyw_event_history));
}

static void event_history_append(u32 type, u32 status, u32 reason, u32 flags)
{
    u32 index;
    if (cyw_event_history.count < CYW_EVENT_HISTORY_CAP) {
        index = (cyw_event_history.first + cyw_event_history.count) %
                CYW_EVENT_HISTORY_CAP;
        cyw_event_history.count++;
    } else {
        index = cyw_event_history.first;
        cyw_event_history.first = (cyw_event_history.first + 1U) %
                                  CYW_EVENT_HISTORY_CAP;
    }

    struct cyw_event_record *record = &cyw_event_history.records[index];
    record->type = type;
    record->status = status;
    record->reason = reason;
    record->flags = flags;
    record->timestamp_ms = timer_monotonic_ms();
    record->reserved = 0U;
}

static void handle_event(const u8 *data, u32 len)
{
    /* BDC data header(4) + Ethernet(14) + Broadcom header(10) +
     * wl_event_msg(48). */
    const u32 event_base = 4U;
    if (len < event_base + 72U)
        return;

    /* wl_event_msg starts after BDC + Ethernet + Broadcom headers:
     * version[0:2], flags[2:4], event_type[4:8], status[8:12]. */
    u16 event_flags = ((u16)data[event_base + 26U] << 8) |
                      (u16)data[event_base + 27U];
    u32 event_type = ((u32)data[event_base + 28U] << 24) |
                     ((u32)data[event_base + 29U] << 16) |
                     ((u32)data[event_base + 30U] << 8) |
                     (u32)data[event_base + 31U];
    u32 status = ((u32)data[event_base + 32U] << 24) |
                 ((u32)data[event_base + 33U] << 16) |
                 ((u32)data[event_base + 34U] << 8) |
                 (u32)data[event_base + 35U];
    u32 reason = ((u32)data[event_base + 36U] << 24) |
                 ((u32)data[event_base + 37U] << 16) |
                 ((u32)data[event_base + 38U] << 8) |
                 (u32)data[event_base + 39U];
    cyw_diag.last_event_type = event_type;
    cyw_diag.last_event_status = status;
    cyw_diag.last_event_flags = event_flags;
    cyw_diag.last_event_len = len;
    cyw_diag.last_event_reason = reason;
    cyw_diag.event_count++;
    event_history_append(event_type, status, reason, event_flags);
    u32 capture = len < 36U ? len : 36U;
    memset(cyw_diag.event_words, 0, sizeof(cyw_diag.event_words));
    for (u32 i = 0U; i < capture; i++)
        ((u8 *)cyw_diag.event_words)[i] = data[i];

    switch (event_type) {
    case CYW_E_AUTH:
        if (status != CYW_E_STATUS_SUCCESS)
            cyw_link = CYW_LINK_AUTH_FAIL;
        break;

    case CYW_E_ASSOC:
        if (status != CYW_E_STATUS_SUCCESS)
            cyw_link = CYW_LINK_AUTH_FAIL;
        break;

    case CYW_E_PSK_SUP:
        if (status == CYW_E_STATUS_TIMEOUT) {
            cyw_link = CYW_LINK_AUTH_FAIL;
        }
        break;

    case CYW_E_SET_SSID:
        if (status == CYW_E_STATUS_SUCCESS) {
            if (!wpa_host.enabled || wpa_host.keys_installed)
                cyw_link = CYW_LINK_UP;
            uart_puts("[cyw] connected\n");
        } else {
            cyw_link = CYW_LINK_AUTH_FAIL;
            uart_puts("[cyw] conn fail st=");
            uart_hex(status);
            uart_puts("\n");
        }
        break;

    case CYW_E_LINK:
        if (status == CYW_E_STATUS_SUCCESS) {
            if (event_flags & 1U) {
                if (!wpa_host.enabled || wpa_host.keys_installed)
                    cyw_link = CYW_LINK_UP;
                uart_puts("[cyw] link up\n");
            } else {
                cyw_link = CYW_LINK_DOWN;
                uart_puts("[cyw] link down\n");
            }
        }
        break;

    case CYW_E_DISASSOC_IND:
    case CYW_E_DEAUTH_IND:
        cyw_link = CYW_LINK_DOWN;
        uart_puts("[cyw] discon ev=");
        uart_hex(event_type);
        uart_puts(")\n");
        break;

    case CYW_E_ESCAN_RESULT:
        if (status == CYW_E_STATUS_PARTIAL) {
            /* Event payload starts after the 76-byte wrapped event. The
             * escan_result fixed header is 12
             * bytes, followed by packed brcmf_bss_info. */
            const u32 bss = event_base + 84U;
            if (bss + 8U <= len) {
                u32 record_len = load_le32(data + bss + 4U);
                if (record_len <= len - bss)
                    (void)scan_store_bss(data + bss, record_len);
            }
        } else if (status == CYW_E_STATUS_SUCCESS) {
            scan_in_progress = false;
            scan_results_pending = false;
            scan_kicks_remaining = 0U;
            uart_puts("[cyw] scan done n=");
            uart_hex(scan_count);
            uart_puts("\n");
        }

        break;

    default:
        break;
    }
}

static u32 load_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) |
           ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void store_le32(u8 *p, u32 value)
{
    p[0]=(u8)value;
    p[1]=(u8)(value>>8);
    p[2]=(u8)(value>>16);
    p[3]=(u8)(value>>24);
}

static bool scan_store_bss(const u8 *bss, u32 record_len)
{
    if (!bss || record_len < 128U)
        return false;
    u8 ssid_len = bss[18U];
    if (ssid_len > CYW_SSID_MAX ||
        19U + (u32)ssid_len > record_len)
        return false;

    u32 slot = scan_count;
    for (u32 i = 0U; i < scan_count; i++) {
        if (memcmp(scan_results[i].bssid, bss + 8U, CYW_MAC_LEN) == 0) {
            slot = i;
            break;
        }
    }
    if (slot == scan_count) {
        if (scan_count >= CYW_MAX_SCAN_RESULTS)
            return true;
        scan_count++;
        memset(&scan_results[slot], 0, sizeof(scan_results[slot]));
    }

    struct cyw_scan_result *r = &scan_results[slot];
    memcpy(r->bssid, bss + 8U, CYW_MAC_LEN);
    r->capability = (u16)bss[16U] | ((u16)bss[17U] << 8);
    r->ssid_len = ssid_len;
    memcpy(r->ssid, bss + 19U, ssid_len);
    r->rssi = (i16)((u16)bss[78U] | ((u16)bss[79U] << 8));
    r->chanspec = (u16)bss[72U] | ((u16)bss[73U] << 8);
    r->channel = bss[88U];
    r->security = 0U;
    u16 ie_offset = (u16)bss[116U] | ((u16)bss[117U] << 8);
    u32 ie_length = load_le32(bss + 120U);
    if (ie_offset <= record_len && ie_length <= record_len - ie_offset) {
        const u8 *ie = bss + ie_offset;
        u32 available = ie_length;
        while (available >= 2U) {
            u32 item_len = ie[1];
            if (item_len + 2U > available)
                break;
            if (ie[0] == 48U && item_len >= 8U) {
                u32 rsn_len = item_len + 2U;
                if (rsn_len <= sizeof(r->rsn_ie)) {
                    memcpy(r->rsn_ie, ie, rsn_len);
                    r->rsn_ie_len = (u8)rsn_len;
                }
                const u8 *p = ie + 2U;
                u32 left = item_len;
                if (left < 8U)
                    break;
                p += 6U;
                left -= 6U;
                u16 pairwise_count = (u16)p[0] | ((u16)p[1] << 8);
                p += 2U;
                left -= 2U;
                u32 pairwise_bytes = (u32)pairwise_count * 4U;
                if (pairwise_bytes > left)
                    break;
                p += pairwise_bytes;
                left -= pairwise_bytes;
                if (left < 2U)
                    break;
                u16 akm_count = (u16)p[0] | ((u16)p[1] << 8);
                p += 2U;
                left -= 2U;
                u32 akm_bytes = (u32)akm_count * 4U;
                if (akm_bytes > left)
                    break;
                for (u32 i = 0U; i < akm_count; i++, p += 4U) {
                    if (p[0] != 0x00U || p[1] != 0x0FU ||
                        p[2] != 0xACU)
                        continue;
                    if (p[3] == 2U)
                        r->security |= CYW_SCAN_SEC_WPA2_PSK;
                    else if (p[3] == 6U)
                        r->security |= CYW_SCAN_SEC_PSK_SHA256;
                    else if (p[3] == 8U)
                        r->security |= CYW_SCAN_SEC_SAE;
                }
                left -= akm_bytes;
                if (left >= 2U) {
                    u16 caps = (u16)p[0] | ((u16)p[1] << 8);
                    if ((caps & (1U << 7)) != 0U)
                        r->security |= CYW_SCAN_SEC_MFP_CAPABLE;
                    if ((caps & (1U << 6)) != 0U)
                        r->security |= CYW_SCAN_SEC_MFP_REQUIRED;
                }
                break;
            }
            ie += item_len + 2U;
            available -= item_len + 2U;
        }
    }
    cyw_diag.scan_result_count = scan_count;
    return true;
}

static bool scan_parse_legacy_results(const u8 *data, u32 len)
{
    if (!data || len < 12U)
        return false;

    u32 buflen = load_le32(data);
    u32 records = load_le32(data + 8U);
    if (buflen < 12U)
        return false;
    bool truncated = buflen > len;
    if (buflen < len)
        len = buflen;

    u32 offset = 12U;
    scan_count = 0U;
    for (u32 i = 0U; i < records; i++) {
        if (offset > len || len - offset < 8U) {
            if (truncated && scan_count > 0U)
                break;
            return false;
        }
        const u8 *bss = data + offset;
        u32 record_len = load_le32(bss + 4U);
        if (record_len < 128U)
            return false;
        if (record_len > len - offset) {
            if (truncated && scan_count > 0U)
                break;
            return false;
        }
        if (!scan_store_bss(bss, record_len))
            return false;
        offset += record_len;
    }
    return true;
}

static bool scan_result_request_start(void)
{
    static u8 buf[BCDC_HEADER_LEN + CYW_SCAN_QUERY_BYTES] ALIGNED(64);
    if (!sdpcm_can_send(SDPCM_CTL_CHANNEL))
        return false;

    memset(scan_result_buf, 0, sizeof(scan_result_buf));
    scan_result_buf[0] = (u8)(sizeof(scan_result_buf) & 0xFFU);
    scan_result_buf[1] = (u8)((sizeof(scan_result_buf) >> 8) & 0xFFU);
    scan_result_buf[4] = 109U;

    u16 id = bcdc_reqid++;
    u32 cmd = WLC_SCAN_RESULTS;
    buf[0] = (u8)cmd;
    buf[1] = (u8)(cmd >> 8);
    buf[2] = (u8)(cmd >> 16);
    buf[3] = (u8)(cmd >> 24);
    buf[4] = (u8)sizeof(scan_result_buf);
    buf[5] = (u8)(sizeof(scan_result_buf) >> 8);
    buf[6] = 0U;
    buf[7] = 0U;
    buf[8] = 0U;
    buf[9] = 0U;
    buf[10] = (u8)id;
    buf[11] = (u8)(id >> 8);
    memset(buf + 12U, 0, 4U);
    memcpy(buf + BCDC_HEADER_LEN, scan_result_buf, sizeof(scan_result_buf));
    if (!sdpcm_send(SDPCM_CTL_CHANNEL, buf, sizeof(buf)))
        return false;

    scan_result_request_id = id;
    scan_result_request_pending = true;
    scan_result_request_attempts++;
    scan_result_request_deadline_ms =
        timer_monotonic_ms() + CYW_BCDC_GET_TIMEOUT_MS;
    return true;
}

static void scan_bus_kick(void)
{
    /* Never block: this runs from the reactor and from the association poll
     * loop, so it must fail closed when the firmware transmit window is
     * exhausted rather than entering sdpcm_send()'s credit wait. */
    /* Preserve the last advertised TX credit for the actual result request.
     * Kicks are optional progress nudges; consuming the final credit makes
     * result retrieval impossible if the firmware has no event to publish. */
    if (!sdpcm_can_send(SDPCM_CTL_CHANNEL) ||
        (u8)(cyw_tx_max - cyw_tx_seq) <= 1U)
        return;

    /* The firmware publishes queued SDPCM frames in response to host bus
     * activity. The previous implementation used WLC_UP for this, which
     * reopens the radio (`wl_open`) and disrupts the scan in progress.
     * Re-sending the idempotent event mask is RF-neutral: it pokes the bus
     * without changing radio, channel or scan state. */
    u8 evmask[16];
    memset(evmask, 0, sizeof(evmask));
    const u32 events[] = {
        CYW_E_SET_SSID, CYW_E_JOIN, CYW_E_AUTH, CYW_E_DEAUTH,
        CYW_E_DEAUTH_IND, CYW_E_ASSOC, CYW_E_DISASSOC,
        CYW_E_DISASSOC_IND, CYW_E_LINK, CYW_E_PSK_SUP,
        CYW_E_ESCAN_RESULT
    };
    for (u32 i = 0U; i < sizeof(events) / sizeof(events[0]); i++)
        evmask[events[i] / 8U] |= (u8)(1U << (events[i] % 8U));
    (void)bcdc_set_iovar("event_msgs", evmask, sizeof(evmask), false);
}

static void scan_result_poll(void)
{
    if (scan_result_request_pending) {
        u32 len = sizeof(scan_result_buf);
        u32 status = 0U;
        if (bcdc_take_response(scan_result_request_id, scan_result_buf,
                               &len, &status)) {
            scan_result_request_pending = false;
            if (status == 0U && scan_parse_legacy_results(scan_result_buf, len)) {
                scan_result_request_attempts = 0U;
                scan_in_progress = false;
                scan_results_pending = false;
            } else if (scan_result_request_attempts < 3U) {
                scan_ready_ms = timer_monotonic_ms() + 250ULL;
                scan_next_kick_ms = timer_monotonic_ms();
            } else {
                scan_in_progress = false;
                scan_results_pending = false;
            }
            return;
        }
        if (timer_monotonic_ms() >= scan_result_request_deadline_ms) {
            scan_result_request_pending = false;
            cyw_diag.bcdc_timeouts++;
            if (scan_result_request_attempts < 3U) {
                scan_ready_ms = timer_monotonic_ms() + 250ULL;
                if (scan_kicks_remaining == 0U)
                    scan_kicks_remaining = 4U;
                scan_next_kick_ms = timer_monotonic_ms();
            } else {
                scan_in_progress = false;
                scan_results_pending = false;
            }
        }
        return;
    }
    if (scan_in_progress && timer_monotonic_ms() >= scan_ready_ms) {
        if (scan_result_request_start())
            return;
    }
    /* While the scan runs, poke the bus on a bounded cadence so the firmware
     * flushes queued escan event frames to the host. */
    if (scan_in_progress && scan_kicks_remaining != 0U &&
        timer_monotonic_ms() >= scan_next_kick_ms) {
        scan_kicks_remaining--;
        scan_next_kick_ms = timer_monotonic_ms() + CYW_SCAN_KICK_INTERVAL_MS;
        scan_bus_kick();
    }
    /* The association window needs the same RF-neutral poke: without it the
     * firmware queues its AUTH/ASSOC/LINK/PSK_SUP events without publishing
     * them, so the host observes an association that never reports progress. */
    if (cyw_link == CYW_LINK_JOINING && join_kicks_remaining != 0U &&
        timer_monotonic_ms() >= join_next_kick_ms) {
        join_kicks_remaining--;
        join_next_kick_ms = timer_monotonic_ms() + CYW_SCAN_KICK_INTERVAL_MS;
        scan_bus_kick();
    }
}

/* ── Firmware upload ── */

/* upload_nvram is called from cyw43_load_firmware below */

static bool upload_nvram(const u8 *nvram, u32 nvram_len)
{
    uart_puts("[cyw] uploading NVRAM (");
    uart_hex(nvram_len);
    uart_puts(" B)...\n");

    /* NVRAM goes at the end of RAM, with a length token */
    u32 ram_size = cyw_ram_bytes;
    if (ram_size == 0U)
        return false;

    /* Condense NVRAM: strip comments/blanks, NUL-separate key=value pairs */
    static u8 nvram_condensed[4096];
    u32 clen = 0;
    for (u32 i = 0; i < nvram_len; ) {
        /* Skip comment lines */
        if (nvram[i] == '#') {
            while (i < nvram_len && nvram[i] != '\n') i++;
            if (i < nvram_len) i++;
            continue;
        }
        /* Skip blank lines */
        if (nvram[i] == '\n' || nvram[i] == '\r') { i++; continue; }
        /* Copy key=value until newline, terminate with NUL */
        while (i < nvram_len && nvram[i] != '\n' && nvram[i] != '\r'
               && clen < sizeof(nvram_condensed) - 2)
            nvram_condensed[clen++] = nvram[i++];
        nvram_condensed[clen++] = '\0';
        while (i < nvram_len && (nvram[i] == '\n' || nvram[i] == '\r')) i++;
    }
    nvram_condensed[clen++] = '\0'; /* double-NUL terminator */

    /* Pad to 4-byte boundary */
    while (clen & 3) nvram_condensed[clen++] = '\0';

    u32 nvram_offset = ram_size - 4 - clen;
    nvram_offset &= ~0x3U;  /* word-align */

    if (!bp_write_buf(cyw_ram_base + nvram_offset, nvram_condensed, clen)) {
        uart_puts("[cyw] NVRAM write fail\n");
        return false;
    }

    /* Write length token: complement of size-in-words in upper 16 bits */
    u32 token = (~(clen / 4) << 16) | (clen / 4);
    if (!bp_write32(cyw_ram_base + ram_size - 4, token)) {
        uart_puts("[cyw] NVRAM token fail\n");
        return false;
    }

    uart_puts("[cyw] nvram ");
    uart_hex(clen);
    uart_puts("B\n");
    return true;
}

/* ── Backplane init (Issue #65) ── */

static bool cyw43_backplane_init(void)
{
    /* ALP clock MUST be active before accessing core wrappers */
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, 0);
    delay_cycles(5000);
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR,
                     CLKCSR_Nohwreq | CLKCSR_ReqALP);
    for (u32 i = 0; i < 1000; i++) {
        u8 clk;
        if (sdio_cmd52_read(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, &clk) &&
            (clk & (CLKCSR_ALPavail | CLKCSR_HTavail)))
            break;
        delay_cycles(5000);
    }
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR,
                     CLKCSR_Nohwreq | CLKCSR_ForceALP);
    delay_cycles(30000);
    uart_puts("[cyw] ALP ok\n");

    /* Core scan: read EROM to discover actual core addresses */
    u32 eromptr;
    if (!bp_read32(CYW_CHIPCOMMON_BASE + 0xFC, &eromptr)) {
        uart_puts("[cyw] EROMPTR fail\n");
        return false;
    }
    uart_puts("[cyw] EROM=");
    uart_hex(eromptr);
    uart_puts("\n");

    /* Parse EROM entries to find ARM, D11, SOCSRAM, SDIOD cores */
    u32 arm_ctl = 0, arm_regs = 0, d11_ctl = 0;
    u32 sram_ctl = 0, sram_regs = 0, sdio_regs = 0;
    u32 coreid = 0;
    u32 core_rev = 0;
    u32 arm_core_id = 0;
    for (u32 i = 0; i < 512; i += 4) {
        u32 entry;
        if (!bp_read32(eromptr + i, &entry))
            break;
        u32 tag = entry & 0xF;
        if (tag == 0xF) break;  /* end marker */
        if (tag == 0x1) {  /* component info */
            u32 next;
            if (!bp_read32(eromptr + i + 4, &next)) break;
            if ((next & 0xF) == 0x1) {
                coreid = (entry >> 8) & 0xFFF;
                core_rev = (next >> 24) & 0xFFU;
                uart_puts("[e] c=");
                uart_hex(coreid);
                i += 4;
            }
        } else if (tag == 0x5) {  /* address descriptor */
            u32 addr = entry & 0xFFFFF000U;
            bool is_ctl = (entry & 0xC0) != 0;
            uart_puts(is_ctl ? " W" : " M");
            uart_hex(addr);
            switch (coreid) {
            case 0x82A: case 0x83C: case 0x83E: case 0x847:  /* ARM CM3 / CR4 / CA7 */
                if (is_ctl && !arm_ctl) arm_ctl = addr;
                if (!is_ctl && !arm_regs) arm_regs = addr;
                arm_core_id = coreid;
                break;
            case 0x80E: case 0x135:  /* SOCSRAM / SOCRAM-es */
                if (is_ctl && !sram_ctl) sram_ctl = addr;
                if (!is_ctl && !sram_regs) sram_regs = addr;
                cyw_sram_rev = core_rev;
                break;
            case 0x812:  /* D11 */
                if (is_ctl && !d11_ctl) d11_ctl = addr;
                break;
            case 0x829:  /* SDIOD */
                if (!is_ctl && !sdio_regs) sdio_regs = addr;
                break;
            }
        }
    }

    uart_puts("[cyw] ARM=");
    uart_hex(arm_ctl);
    uart_puts(" r=");
    uart_hex(arm_regs);
    uart_puts(" SRAM=");
    uart_hex(sram_ctl);
    uart_puts(" SDIO=");
    uart_hex(sdio_regs);
    uart_puts("\n");

    if (!arm_ctl) {
        uart_puts("[cyw] no ARM core\n");
        return false;
    }

    /* Store discovered addresses for firmware load */
    cyw_arm_ctl = arm_ctl;
    cyw_arm_regs = arm_regs;
    cyw_arm_core_id = arm_core_id;
    cyw_d11_ctl = d11_ctl;
    cyw_sram_ctl = sram_ctl;
    cyw_sram_regs = sram_regs;
    cyw_sdio_regs = sdio_regs;
    if (cyw_arm_core_id == 0x82AU && (!cyw_sram_ctl || !cyw_sram_regs)) {
        uart_puts("[cyw] CM3 requires SOCRAM\n");
        return false;
    }

    cyw_ram_bytes = cyw43_firmware_ramsize();
    if (cyw_ram_bytes == 0U || cyw_ram_bytes > 4U * 1024U * 1024U) {
        uart_puts("[cyw] invalid RAM size\n");
        return false;
    }
    uart_puts("[cyw] RAM base=");
    uart_hex(cyw_ram_base);
    uart_puts(" size=");
    uart_hex(cyw_ram_bytes);
    uart_puts("\n");

    /* Match brcmf_chip_disable_arm(): CR4 stays halted, while CM3 is
     * disabled with its normal wrapper reset sequence. */
    if (((cyw_arm_core_id == 0x83CU || cyw_arm_core_id == 0x83EU) &&
         !core_reset(cyw_arm_ctl, SICF_CPUHALT,
                     SICF_CPUHALT, SICF_CPUHALT)) ||
        (cyw_arm_core_id != 0x83CU && cyw_arm_core_id != 0x83EU &&
         !core_disable(cyw_arm_ctl, 0U, 0U))) {
        uart_puts("[cyw] ARM!\n");
        return false;
    }

    /* Match brcmf_chip_cr4_set_passive(): D11 remains disabled in reset;
     * firmware, not the host, releases and configures it. */
    if (cyw_d11_ctl &&
        !core_disable(cyw_d11_ctl,
                      D11_PHYRESET | D11_PHYCLOCKEN,
                      D11_PHYCLOCKEN)) {
        uart_puts("[cyw] D11!\n");
        return false;
    }

    /* Clear pull-ups/pull-downs */
    if (!sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_PULLUPS, 0) ||
        !bp_write32(CYW_CHIPCOMMON_BASE + CC_GPIOPULLUP, 0) ||
        !bp_write32(CYW_CHIPCOMMON_BASE + CC_GPIOPULLDOWN, 0)) {
        uart_puts("[cyw] GPIO pulls fail\n");
        return false;
    }

    /* Do not write ChipControl index 1 on CYW43455. Circle's reference
     * driver applies the bits 13:11 SDIO drive-strength write only to the
     * older 4330/43362 parts and returns before it for 4345. On the Pi 5
     * CYW43455, forcing 0b111 there prevents the post-firmware HT clock from
     * becoming available (stage 23), so the safe target-specific behavior is
     * to leave the firmware/platform pad configuration unchanged. */

    return true;
}

/* ── Public API ── */

bool cyw43_init(void)
{
    cyw_diag.stage = 1U;
    cyw_diag.last_error = 0U;
    cyw_link = CYW_LINK_DOWN;
    cyw_arm_ctl = 0U;
    cyw_arm_regs = 0U;
    cyw_arm_core_id = 0U;
    cyw_d11_ctl = 0U;
    cyw_sram_ctl = 0U;
    cyw_sram_regs = 0U;
    cyw_sram_rev = 0U;
    cyw_sdio_regs = 0U;
    cyw_ram_base = CYW_RAM_BASE;
    cyw_ram_bytes = 0U;
    cyw_chip_id = 0U;
    cyw_tx_seq = 0;
    cyw_tx_max = 4U;
    cyw_tx_fcmask = 0U;
    cyw_backplane_window = 0;
    sdpcm_frame_pending = false;
    sdpcm_next_len = 0U;
    bcdc_reqid = 1;
    bcdc_ignore_through = 0U;
    memset(bcdc_responses, 0, sizeof(bcdc_responses));
    cyw_data_rx_head = 0U;
    cyw_data_rx_tail = 0U;
    cyw_data_rx_count = 0U;
    cyw_diag.data_rx_queued = 0U;
    cyw_diag.data_rx_dropped = 0U;
    /* Preserve the last completed scan cache across a clean firmware
     * reinitialization. Association needs fresh SDPCM credits after scanning,
     * but still needs the selected BSSID/chanspec from that scan. A new
     * cyw43_scan_start() remains the authoritative cache reset. */
    cyw_diag.scan_result_count = scan_count;
    scan_in_progress = false;
    scan_results_pending = false;
    scan_ready_ms = 0ULL;
    scan_result_request_pending = false;
    scan_kicks_remaining = 0U;
    scan_next_kick_ms = 0ULL;
    join_kicks_remaining = 0U;
    join_next_kick_ms = 0ULL;
    cyw_diag.last_event_type = 0U;
    cyw_diag.last_event_status = 0U;
    cyw_diag.last_event_flags = 0U;
    cyw_diag.last_event_len = 0U;
    cyw_diag.event_count = 0U;
    cyw_diag.scan_result_count = scan_count;
    cyw_diag.last_event_reason = 0U;
    memset(cyw_diag.event_words, 0, sizeof(cyw_diag.event_words));
    event_history_reset();
    cyw_eapol_len = 0U;
    cyw_m2_len = 0U;
    cyw_diag.eapol_frames = 0U;
    cyw_diag.eapol_len = 0U;
    cyw_diag.eapol_key_info = 0U;
    cyw_diag.eapol_replay_hi = 0U;
    cyw_diag.eapol_replay_lo = 0U;
    cyw_diag.eapol_m2_sent = 0U;
    cyw_diag.eapol_m2_fail = 0U;
    memset(cyw_diag.eapol_words, 0, sizeof(cyw_diag.eapol_words));
    memset(cyw_mac, 0, CYW_MAC_LEN);
    memset(&wpa_host, 0, sizeof(wpa_host));

    uart_puts("[cyw] init...\n");

    /* Initialize SDIO controller and enumerate card */
    if (!sdio_init()) {
        cyw_diag.last_error = 1U;
        uart_puts("[cyw] SDIO fail\n");
        return false;
    }
    /* Arm the BCM2712 SDIO2 level interrupt before enabling the CYW functions.
     * From this point onward the reactor can sleep until DAT1 signals work;
     * the slow speculative probe remains only as a missed-IRQ safety net. */
    cyw43_register_sdio_irq();

    /* Enable backplane function (func 1) */
    if (!sdio_enable_func(SDIO_FUNC_BACKPLANE)) {
        cyw_diag.last_error = 2U;
        uart_puts("[cyw] f1 enable fail\n");
        return false;
    }
    sdio_set_block_size(SDIO_FUNC_BACKPLANE, SDIO_FUNC1_BLKSZ);

    /* Enable 4-bit SDIO bus */
    sdio_set_bus_width_4bit();

    /* Identify chip via backplane */
    if (!chip_identify()) {
        cyw_diag.last_error = 3U;
        uart_puts("[cyw] chip ID fail\n");
        return false;
    }

    /* Backplane init: halt ARM, reset cores, setup clocks */
    if (!cyw43_backplane_init()) {
        cyw_diag.last_error = 4U;
        uart_puts("[cyw] bp fail\n");
        return false;
    }

    /* Enable function interrupts for backplane */
    sdio_enable_func_irq(SDIO_FUNC_BACKPLANE);

    /* NOTE: func 2 (WLAN) is enabled AFTER firmware load in cyw43_load_firmware(),
     * because the WLAN function won't be ready until firmware boots. */

    uart_puts("[cyw] init OK\n");
    cyw_diag.stage = 2U;
    return true;
}

bool cyw43_preload_blobs(void)
{
    if (blobs_loaded) {
        uart_puts("[cyw-pre] using installed blobs fw=");
        uart_hex(fw_buf_len);
        uart_puts(" nv=");
        uart_hex(nvram_buf_len);
        uart_puts(" clm=");
        uart_hex(clm_buf_len);
        uart_puts("\n");
        return true;
    }

    cyw_diag.stage = 10U;
    fw_buf_len = nvram_buf_len = clm_buf_len = 0;
    fw_data = fw_buf;
    nvram_data = nvram_buf;
    clm_data = clm_buf;

    if (!fat32_init()) {
        uart_puts("[cyw-pre] FAT32 fail\n");
        return false;
    }

    const char *fw_path = "/wifi/firmware.bin";
    const char *nv_path = "/wifi/nvram.txt";
    const char *clm_path = "/wifi/clm.bin";
#if PIOS_PLATFORM == PIOS_PLATFORM_PI3
    if (cyw43_board_model() == BOARD_MODEL_PI3_B_PLUS) {
        fw_path = "/wifi/pi3bp/firmware.bin";
        nv_path = "/wifi/pi3bp/nvram.txt";
        clm_path = "/wifi/pi3bp/clm.bin";
    } else {
        fw_path = "/wifi/pi3b/firmware.bin";
        nv_path = "/wifi/pi3b/nvram.txt";
        clm_path = "/wifi/pi3b/clm.bin";
    }
#elif PIOS_PLATFORM == PIOS_PLATFORM_PIZERO2W
    fw_path = "/wifi/zero2w/firmware.bin";
    nv_path = "/wifi/zero2w/nvram.txt";
    clm_path = "/wifi/zero2w/clm.bin";
#endif

    /* Firmware */
    {
        fat32_file_t fw;
        if (!fat32_open(fw_path, &fw)) {
            uart_puts("[cyw-pre] no fw\n");
            return false;
        }
        if (fw.file_size > CYW_FW_MAX_SIZE) {
            uart_puts("[cyw-pre] fw too big\n");
            fat32_close(&fw);
            return false;
        }
        u32 off = 0;
        while (off < fw.file_size) {
            u32 chunk = fw.file_size - off;
            if (chunk > 4096) chunk = 4096;
            u32 got = fat32_read(&fw, fw_buf + off, chunk);
            if (got == 0) {
                uart_puts("[cyw-pre] fw read err @");
                uart_hex(off);
                uart_puts("\n");
                fat32_close(&fw);
                return false;
            }
            off += got;
        }
        fat32_close(&fw);
        fw_buf_len = off;
        uart_puts("[cyw-pre] fw loaded ");
        uart_hex(fw_buf_len);
        uart_puts("\n");
    }

    /* NVRAM (optional) */
    {
        fat32_file_t nv;
        if (fat32_open(nv_path, &nv)) {
            if (nv.file_size <= CYW_NVRAM_MAX) {
                nvram_buf_len = fat32_read(&nv, nvram_buf, nv.file_size);
                uart_puts("[cyw-pre] nvram loaded ");
                uart_hex(nvram_buf_len);
                uart_puts("\n");
            }
            fat32_close(&nv);
        }
    }

    /* CLM (optional) */
    {
        fat32_file_t clm;
        if (fat32_open(clm_path, &clm)) {
            if (clm.file_size <= CYW_CLM_MAX) {
                clm_buf_len = fat32_read(&clm, clm_buf, clm.file_size);
                uart_puts("[cyw-pre] clm loaded ");
                uart_hex(clm_buf_len);
                uart_puts("\n");
            }
            fat32_close(&clm);
        }
    }

    blobs_loaded = fw_buf_len != 0U && nvram_buf_len != 0U &&
                   clm_buf_len != 0U;
    cyw_diag.fw_len = fw_buf_len;
    cyw_diag.nvram_len = nvram_buf_len;
    cyw_diag.clm_len = clm_buf_len;
    if (!blobs_loaded) {
        cyw_diag.last_error = 10U;
        uart_puts("[cyw-pre] incomplete blob set\n");
        return false;
    }
    cyw_diag.stage = 11U;
    return true;
}

bool cyw43_load_firmware(void)
{
    cyw_diag.stage = 20U;
    cyw_diag.last_error = 0U;
    cyw_diag.ram_base = cyw_ram_base;
    cyw_diag.ram_size = cyw_ram_bytes;
    cyw_diag.firmware_uploaded = 0U;
    cyw_diag.firmware_verified = 0U;
    cyw_diag.f1_batch_blocks = 0U;
    cyw_diag.firmware_upload_ms = 0U;
    cyw_diag.ht_available = 0U;
    cyw_diag.func2_ready = 0U;
    cyw_diag.clm_loaded = 0U;
    /*
     * Load firmware, NVRAM, and CLM blobs from preloaded RAM buffers
     * (cyw43_preload_blobs() must be called BEFORE cyw43_init since
     *  SDIO2 init disturbs the EMMC2 SD controller).
     */

    if (!blobs_loaded) {
        cyw_diag.last_error = 20U;
        uart_puts("[cyw] blobs not preloaded\n");
        return false;
    }
    uart_puts("[cyw] using preloaded blobs fw=");
    uart_hex(fw_buf_len);
    uart_puts(" nv=");
    uart_hex(nvram_buf_len);
    uart_puts(" clm=");
    uart_hex(clm_buf_len);
    uart_puts("\n");

    if (!probe_f1_multiblock()) {
        cyw_diag.last_error = 27U;
        uart_puts("[cyw] F1 multiblock verify failed\n");
        return false;
    }
    cyw_diag.f1_batch_blocks = CYW_F1_BATCH_BLOCKS;
    uart_puts("[cyw] F1 multiblock verified\n");

    /* Request ALP clock for backplane memory access */
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, CLKCSR_ReqALP);
    for (u32 i = 0; i < 1000; i++) {
        u8 clk;
        if (sdio_cmd52_read(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, &clk) &&
            (clk & CLKCSR_ALPavail))
            break;
        delay_cycles(5000);
    }

    /* Quick sanity: try writing 4 bytes at RAM base via bp_write32 */
    uart_puts("[cyw] sanity bp_write32 @ rambase ");
    uart_hex(cyw_ram_base);
    uart_puts("...\n");
    if (!bp_write32(cyw_ram_base, 0xDEADBEEF)) {
        uart_puts("[cyw] sanity bp_write32 @ rambase FAILED\n");
        return false;
    }
    uart_puts("[cyw] sanity bp_write32 @ rambase OK\n");
    {
        u32 rb = 0;
        if (bp_read32(cyw_ram_base, &rb)) {
            uart_puts("[cyw] readback rambase=");
            uart_hex(rb);
            uart_puts("\n");
        } else {
            uart_puts("[cyw] readback FAIL\n");
        }
    }

    /* Clear end of RAM before upload */
    uart_puts("[cyw] clear EOR @");
    uart_hex(cyw_ram_base + cyw_ram_bytes - 4U);
    uart_puts("\n");
    if (!bp_write32(cyw_ram_base + cyw_ram_bytes - 4U, 0)) {
        uart_puts("[cyw] EOR clear FAIL\n");
        return false;
    }
    uart_puts("[cyw] EOR clear OK\n");

    /* Capture reset vector (first 4 bytes of FW) for CR4 */
    u32 resetvec = (u32)fw_data[0] | ((u32)fw_data[1] << 8) |
                   ((u32)fw_data[2] << 16) | ((u32)fw_data[3] << 24);
    cyw_diag.reset_vector = resetvec;

    /* Upload firmware binary from preloaded buffer */
    {
        u64 upload_start_ms = timer_monotonic_ms();
        uart_puts("[cyw] fw (");
        uart_hex(fw_buf_len);
        uart_puts(" bytes)...\n");

        u32 offset = 0;
        while (offset < fw_buf_len) {
            u32 chunk = fw_buf_len - offset;
            if (chunk > CYW_F1_BATCH_BYTES)
                chunk = CYW_F1_BATCH_BYTES;
            if (!bp_write_buf(cyw_ram_base + offset, fw_data + offset, chunk)) {
                uart_puts("[cyw] fw wr err @");
                uart_hex(offset);
                uart_puts("\n");
                return false;
            }
            offset += chunk;
            if ((offset & 0xFFFF) == 0) { uart_putc('.'); }
        }
        uart_puts("[cyw] fw uploaded\n");
        cyw_diag.firmware_uploaded = 1U;
        cyw_diag.firmware_upload_ms =
            (u32)(timer_monotonic_ms() - upload_start_ms);
        cyw_diag.stage = 21U;
        for (u32 sample = 0; sample < fw_buf_len; sample += 0x10000U) {
            u32 got = 0;
            u32 expected = (u32)fw_data[sample] |
                           ((u32)fw_data[sample + 1U] << 8) |
                           ((u32)fw_data[sample + 2U] << 16) |
                           ((u32)fw_data[sample + 3U] << 24);
            if (!bp_read32(cyw_ram_base + sample, &got) ||
                got != expected) {
                uart_puts("[cyw] fw verify fail @");
                uart_hex(sample);
                uart_puts(" got=");
                uart_hex(got);
                uart_puts(" exp=");
                uart_hex(expected);
                uart_puts("\n");
                return false;
            }
        }
        uart_puts("[cyw] fw verify OK\n");
        cyw_diag.firmware_verified = 1U;
        cyw_diag.stage = 22U;
    }

    /* Upload NVRAM from preloaded buffer */
    if (nvram_buf_len > 0) {
        uart_puts("[cyw] nvram (");
        uart_hex(nvram_buf_len);
        uart_puts(" B)\n");
        if (!upload_nvram(nvram_data, nvram_buf_len)) {
            cyw_diag.last_error = 21U;
            uart_puts("[cyw] nvram fail\n");
            return false;
        }
    } else {
        uart_puts("[cyw] nvram def\n");
        static const u8 default_nvram[] =
            "boardtype=0x0646\0"
            "boardrev=0x1101\0"
            "boardflags=0x00404001\0"
            "sromrev=11\0"
            "boardflags3=0x08000188\0"
            "macaddr=00:11:22:33:44:55\0"
            "\0";
        if (!upload_nvram(default_nvram, sizeof(default_nvram)))
            return false;
    }

    /* Verify FW landed in TCM (read first 8 bytes back) */
    {
        u32 v0=0xDEADDEAD, v1=0xDEADDEAD;
        bp_read32(cyw_ram_base, &v0);
        bp_read32(cyw_ram_base + 4, &v1);
        uart_puts("[cyw] fw[0]="); uart_hex(v0);
        uart_puts(" fw[4]="); uart_hex(v1); uart_puts("\n");
    }

    /* Clear SDIOD interrupt status before ARM reset */
    uart_puts("[cyw] post-nvram: clearing SDIOD INTSTATUS\n");
    bp_write32(cyw_sdio_regs + SDIOD_INTSTATUS, 0xFFFFFFFF);

    /* CR4 reset vector: write first 4 bytes of FW to backplane addr 0
     * (ARM CR4 fetches PC=0 on release; this is the trampoline). */
    if (resetvec != 0) {
        uart_puts("[cyw] writing resetvec ");
        uart_hex(resetvec);
        uart_puts(" -> 0\n");
        if (!bp_write32(0, resetvec)) {
            uart_puts("[cyw] resetvec write fail\n");
            return false;
        }
    }

    /* Reset ARM core to start firmware */
    uart_puts("[cyw] ARM reset out-of-halt...\n");
    if (((cyw_arm_core_id == 0x83CU || cyw_arm_core_id == 0x83EU) &&
         !core_reset(cyw_arm_ctl, SICF_CPUHALT, 0U, 0U)) ||
        (cyw_arm_core_id != 0x83CU && cyw_arm_core_id != 0x83EU &&
         !core_reset(cyw_arm_ctl, 0U, 0U, 0U))) {
        cyw_diag.last_error = 22U;
        uart_puts("[cyw] ARM reset fail\n");
        return false;
    }
    uart_puts("[cyw] ARM reset OK\n");

    /* Read CR4 state post-release */
    {
        u32 rc=0xDEADDEAD, ic=0xDEADDEAD;
        bp_read32(cyw_arm_ctl + CORE_RESETCTRL, &rc);
        bp_read32(cyw_arm_ctl + CORE_IOCTRL, &ic);
        cyw_diag.cr4_resetctrl = rc;
        cyw_diag.cr4_ioctrl = ic;
        uart_puts("[cyw] CR4 RESETCTRL="); uart_hex(rc);
        uart_puts(" IOCTRL="); uart_hex(ic); uart_puts("\n");
    }

    /* sbenable: request HT clock and wait (Issue #67) */
    uart_puts("[cyw] CLKCSR=0...\n");
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, 0);
    uart_puts("[cyw] CLKCSR=0 done\n");
    delay_cycles(500000);
    uart_puts("[cyw] CLKCSR=ReqHT...\n");
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, CLKCSR_ReqHT);
    uart_puts("[cyw] CLKCSR=ReqHT done; polling HT...\n");
    bool ht_ok = false;
    u32 ht_polls = 0U;
    u64 ht_deadline = timer_monotonic_ms() + 5000ULL;
    while (timer_monotonic_ms() < ht_deadline) {
        u8 clk = 0;
        bool rd = sdio_cmd52_read(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, &clk);
        cyw_diag.clkcsr = clk;
        if (ht_polls < 3U || (ht_polls & 31U) == 0U) {
            uart_puts("[cyw] HT poll i=");
            uart_hex(ht_polls);
            uart_puts(" rd=");
            uart_hex(rd ? 1 : 0);
            uart_puts(" clk=");
            uart_hex(clk);
            uart_puts("\n");
        }
        if (rd && (clk & CLKCSR_HTavail)) {
            ht_ok = true;
            cyw_diag.ht_available = 1U;
            break;
        }
        ht_polls++;
        if (cyw_progress_hook)
            cyw_progress_hook();
        watchdog_hw_pet();
        timer_delay_ms(10U);
    }
    if (!ht_ok) {
        cyw_diag.last_error = 23U;
        cyw_diag.stage = 23U;
        uart_puts("[cyw] HT timeout\n");
        return false;
    }
    {
        u8 clk;
        sdio_cmd52_read(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, &clk);
        sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR,
                         clk | CLKCSR_ForceHT);
    }
    uart_puts("[cyw] HT ok\n");

    /* Set protocol version */
    bp_write32(cyw_sdio_regs + SDIOD_TOSBMAILBOXDATA, 4 << 16);

    /* Linux HOSTINTMASK: all host-mailbox bits plus CHIPACTIVE. */
    bp_write32(cyw_sdio_regs + SDIOD_INTMASK, 0x200000F0U);

    /* Enable WLAN data function (func 2) — now that firmware is running */
    if (!sdio_enable_func(SDIO_FUNC_WLAN)) {
        cyw_diag.last_error = 24U;
        uart_puts("[cyw] f2 fail\n");
        return false;
    }
    sdio_set_block_size(SDIO_FUNC_WLAN, SDIO_FUNC2_BLKSZ);
    sdio_enable_func_irq(SDIO_FUNC_WLAN);
    cyw_diag.func2_ready = 1U;
    cyw_diag.stage = 24U;

    /* Firmware signals protocol readiness through the host mailbox. Clear
     * SDIOD interrupt status and ACK the mailbox before issuing BCDC. */
    {
        u64 deadline = timer_monotonic_ms() + 3000ULL;
        bool ready = false;
        while (timer_monotonic_ms() < deadline) {
            (void)sdpcm_pending_bytes();
            u32 mailbox = cyw_diag.sdiod_mailbox;
            if (mailbox & (SDIOD_HMB_DEVREADY | SDIOD_HMB_FWREADY)) {
                ready = true;
                break;
            }
            timer_delay_ms(1U);
        }
        if (!ready) {
            cyw_diag.last_error = 25U;
            return false;
        }
    }

    /* Keep the SDIO core awake for protocol traffic. Without KSO the host TX
     * FIFO asserts XMTDATA_AVAIL but firmware never consumes BCDC frames. */
    {
        bool awake = false;
        for (u32 i = 0; i < 300U; i++) {
            u8 sleepcsr = 0;
            if (!sdio_cmd52_write(SDIO_FUNC_BACKPLANE,
                                  SDIO_SLEEPCSR, SLEEPCSR_KSO))
                break;
            timer_delay_us(50U);
            if (sdio_cmd52_read(SDIO_FUNC_BACKPLANE,
                                SDIO_SLEEPCSR, &sleepcsr) &&
                (sleepcsr & (SLEEPCSR_KSO | SLEEPCSR_DEVON)) ==
                    (SLEEPCSR_KSO | SLEEPCSR_DEVON)) {
                awake = true;
                break;
            }
        }
        if (!awake) {
            cyw_diag.last_error = 26U;
            return false;
        }
    }

    /* Re-assert HOSTINTMASK now that function 2 and KSO are up. The value
     * written before F2 was enabled does not survive the firmware's own SDIO
     * core initialization, and a zero mask suppresses the host-mailbox frame
     * indication that publishes queued event frames such as escan results. */
    {
        (void)bp_write32(cyw_sdio_regs + SDIOD_INTMASK, 0x200000F0U);
        (void)bp_read32(cyw_sdio_regs + SDIOD_INTMASK,
                        &cyw_diag.sdiod_intmask);
        uart_puts("[cyw] intmask=");
        uart_hex(cyw_diag.sdiod_intmask);
        uart_puts("\n");
    }

    /* Load CLM blob via 'clmload' iovar from preloaded buffer */
    if (clm_buf_len > 0) {
        uart_puts("[cyw] loading CLM (");
        uart_hex(clm_buf_len);
        uart_puts(" bytes)...\n");

        static u8 ALIGNED(4) clm_chunk[1024 + 16];
        u32 offset = 0;
        bool clm_ok = true;

        while (offset < clm_buf_len && clm_ok) {
            u32 chunk = clm_buf_len - offset;
            /* Keep BCDC+SDPCM below 512 bytes: multi-block F2 transfers are
             * not reliable on BCM2712 SDIO2 during bring-up. */
            if (chunk > 1400U) chunk = 1400U;

            u16 flag = 0x1000U;
            if (offset == 0U) flag |= 0x0002U;
            if (offset + chunk >= clm_buf_len) flag |= 0x0004U;

            clm_chunk[0] = flag & 0xFF;
            clm_chunk[1] = (flag >> 8) & 0xFF;
            clm_chunk[2] = 0x02;
            clm_chunk[3] = 0x00;
            clm_chunk[4] = chunk & 0xFF;
            clm_chunk[5] = (chunk >> 8) & 0xFF;
            clm_chunk[6] = (chunk >> 16) & 0xFF;
            clm_chunk[7] = (chunk >> 24) & 0xFF;
            clm_chunk[8] = 0; clm_chunk[9] = 0;
            clm_chunk[10] = 0; clm_chunk[11] = 0;

            for (u32 i = 0; i < chunk; i++)
                clm_chunk[12 + i] = clm_data[offset + i];

            if (!bcdc_set_iovar("clmload", clm_chunk, 12 + chunk, false))
                clm_ok = false;

            offset += chunk;
        }
        if (clm_ok) {
            cyw_diag.clm_loaded = 1U;
            uart_puts("[cyw] CLM ok\n");
        } else {
            cyw_diag.last_error = 28U;
            uart_puts("[cyw] CLM fail\n");
            return false;
        }
    } else {
        uart_puts("[cyw] no CLM\n");
    }

    /* Use a stable locally-administered address. Leaving cyw_mac zero makes
     * WiFi activation build ARP/Ethernet frames with an invalid source MAC if
     * the synchronous cur_etheraddr GET is delayed or unsupported. */
    {
        static const u8 pios_wifi_mac[CYW_MAC_LEN] =
            { 0x02U, 0x50U, 0x49U, 0x4FU, 0x53U, 0x01U };
        if (!cyw43_set_mac(pios_wifi_mac)) {
            uart_puts("[cyw] MAC set fail\n");
            return false;
        }
    }

    uart_puts("[cyw] fw ok\n");
    cyw_diag.stage = 25U;
    return true;
}

void cyw43_poll(void)
{
    bool active = cyw43_poll_busy();
    u64 now = timer_monotonic_ms();
    bool irq_edge = sdio_card_irq_take();
    if (!active && !irq_edge && now < cyw_poll_idle_next_ms)
        return;
    if (!irq_edge)
        irq_edge = sdio_card_irq_edge();
    cyw_rx_irq_hint = irq_edge;
    if (!active)
        cyw_poll_idle_next_ms = now + 1000ULL;

    /* Drain a bounded burst rather than a single frame. An escan publishes one
     * event frame per BSS plus a completion event; consuming only one frame per
     * reactor pass throttles delivery below the firmware's publication rate and
     * loses results when the queue is serviced too slowly. */
    for (u32 drained = 0U; drained < CYW_POLL_BURST_FRAMES; drained++) {
        if (sdpcm_pending_bytes() == 0U)
            break;

        u8 channel;
        u32 len;
        if (!sdpcm_recv(&channel, cyw_rx_buf, &len))
            break;

        switch (channel) {
        case SDPCM_EVENT_CHANNEL:
            handle_event(cyw_rx_buf, len);
            break;

        case SDPCM_DATA_CHANNEL:
            if (cyw_data_is_eapol(cyw_rx_buf, len))
                capture_eapol(cyw_rx_buf, len);
            else {
                (void)cyw_data_queue_push(cyw_rx_buf, len);
                /*
                 * net_poll() owns regular Ethernet delivery. Do not keep
                 * draining SDPCM data here or a burst fills the bounded queue
                 * before the reactor returns to the network stack.
                 */
                return;
            }
            break;

        case SDPCM_CTL_CHANNEL:
            bcdc_cache_response(cyw_rx_buf, len);
            break;

        default:
            break;
        }
    }
    scan_result_poll();
}

static bool cyw43_radio_enable(void)
{
    u32 zero = 0U;
    u32 one = 1U;
    u32 radio_enable = 1U << 16;
    u32 scan_time = 40U;
    if (!bcdc_set_cmd(WLC_UP, &zero, sizeof(zero), false) ||
        !bcdc_set_cmd(WLC_SET_RADIO, &radio_enable,
                      sizeof(radio_enable), false) ||
        !bcdc_set_cmd(WLC_SET_INFRA, &one, sizeof(one), false) ||
        !bcdc_set_cmd(WLC_SET_PM, &zero, sizeof(zero), false) ||
        !bcdc_set_cmd(WLC_SET_PASSIVE_SCAN, &zero, sizeof(zero), false) ||
        !bcdc_set_cmd(WLC_SET_SCAN_CHANNEL_TIME,
                      &scan_time, sizeof(scan_time), false) ||
        !bcdc_set_cmd(WLC_SET_SCAN_UNASSOC_TIME,
                      &scan_time, sizeof(scan_time), false) ||
        !bcdc_set_iovar("mpc", &zero, sizeof(zero), false))
        return false;

    bool radio_ready = false;
    u64 radio_deadline = timer_monotonic_ms() + 2000ULL;
    while (timer_monotonic_ms() < radio_deadline) {
        u32 resetctrl = AIRC_RESET;
        u32 ioctrl = 0U;
        if (bp_read32(cyw_d11_ctl + CORE_RESETCTRL, &resetctrl) &&
            bp_read32(cyw_d11_ctl + CORE_IOCTRL, &ioctrl) &&
            (resetctrl & AIRC_RESET) == 0U &&
            (ioctrl & SICF_CLOCK_EN) != 0U) {
            radio_ready = true;
            break;
        }
        watchdog_hw_pet();
        timer_delay_ms(10U);
    }
    if (!radio_ready)
        return false;

    u8 evmask[16];
    memset(evmask, 0, sizeof(evmask));
    const u32 events[] = {
        CYW_E_SET_SSID, CYW_E_JOIN, CYW_E_AUTH, CYW_E_DEAUTH,
        CYW_E_DEAUTH_IND, CYW_E_ASSOC, CYW_E_DISASSOC,
        CYW_E_DISASSOC_IND, CYW_E_LINK, CYW_E_PSK_SUP,
        CYW_E_ESCAN_RESULT
    };
    for (u32 i = 0U; i < sizeof(events) / sizeof(events[0]); i++)
        evmask[events[i] / 8U] |= (u8)(1U << (events[i] % 8U));
    if (!bcdc_set_iovar("event_msgs", evmask, sizeof(evmask), false))
        return false;
    timer_delay_ms(100U);
    return true;
}

bool cyw43_scan_start(void)
{
    scan_count = 0;
    scan_in_progress = true;
    scan_results_pending = true;
    /* Keep result retrieval inside the 15-second bounded bus-kick window so
     * the firmware still has opportunities to publish a credit/response. */
    scan_ready_ms = timer_monotonic_ms() + 10000ULL;
    scan_result_request_pending = false;
    scan_result_request_attempts = 0U;
    scan_kicks_remaining = CYW_SCAN_KICKS_MAX;
    scan_next_kick_ms = timer_monotonic_ms() + CYW_SCAN_KICK_INTERVAL_MS;

    if (!cyw43_radio_enable()) {
        scan_in_progress = false;
        scan_results_pending = false;
        return false;
    }

    struct {
        u32 escan_version;
        u16 action;
        u16 sync_id;
        u32 ssid_len;
        u8  ssid[CYW_SSID_MAX];
        u8  bssid[CYW_MAC_LEN];
        i8  bss_type;
        u8  scan_type;
        u32 nprobes;
        u32 active_time;
        u32 passive_time;
        u32 home_time;
        u32 channel_num;
        u16 channel_list[38];
    } PACKED escan;
    _Static_assert(sizeof(escan) == 148U,
                   "Broadcom v1 escan parameters must carry the channel list");

    memset(&escan, 0, sizeof(escan));
    escan.escan_version = 1U;
    escan.action = 1U;
    escan.sync_id = 0x1234U;
    memset(escan.bssid, 0xFF, CYW_MAC_LEN);
    escan.bss_type = 2;  /* any */
    escan.scan_type = 0U; /* active broadcast scan */
    escan.nprobes = 0xFFFFFFFFU;
    escan.active_time = 0xFFFFFFFFU;
    escan.passive_time = 0xFFFFFFFFU;
    escan.home_time = 0xFFFFFFFFU;
    u32 channels = 0U;
    for (u32 channel = 1U; channel <= 13U; channel++)
        escan.channel_list[channels++] = (u16)(0x2B00U | channel);
    static const u8 channels_5g[] = {
        36U, 40U, 44U, 48U, 52U, 56U, 60U, 64U,
        100U, 104U, 108U, 112U, 116U, 120U, 124U, 128U,
        132U, 136U, 140U, 144U, 149U, 153U, 157U, 161U, 165U
    };
    for (u32 i = 0U; i < sizeof(channels_5g); i++)
        escan.channel_list[channels++] = (u16)(0x1B00U | channels_5g[i]);
    escan.channel_num = channels;

    if (!bcdc_set_iovar("escan", &escan, sizeof(escan), false)) {
        scan_in_progress = false;
        scan_results_pending = false;
        return false;
    }
    return true;
}

bool cyw43_scan_get_results(struct cyw_scan_result *results, u32 *count)
{
    if (scan_in_progress || scan_result_request_pending ||
        scan_results_pending)
        return false;

    u32 n = scan_count;
    if (n > *count) n = *count;

    memcpy(results, scan_results, n * sizeof(struct cyw_scan_result));
    *count = n;
    return true;
}

bool cyw43_scan_in_progress(void)
{
    scan_result_poll();
    return scan_in_progress || scan_result_request_pending ||
           scan_results_pending;
}

bool cyw43_radio_query(u32 *radio, u32 *channel)
{
    if (!radio || !channel)
        return false;
    u8 radio_buf[4] = {0};
    u32 radio_len = sizeof(radio_buf);
    if (!bcdc_get_cmd(WLC_GET_RADIO, radio_buf,
                      sizeof(radio_buf), &radio_len) ||
        radio_len < 4U)
        return false;

    u8 channel_buf[12] = {0};
    u32 channel_len = sizeof(channel_buf);
    if (!bcdc_get_cmd(WLC_GET_CHANNEL, channel_buf,
                      sizeof(channel_buf), &channel_len) ||
        channel_len < 4U)
        return false;

    *radio = (u32)radio_buf[0] | ((u32)radio_buf[1] << 8) |
             ((u32)radio_buf[2] << 16) | ((u32)radio_buf[3] << 24);
    *channel = (u32)channel_buf[0] | ((u32)channel_buf[1] << 8) |
               ((u32)channel_buf[2] << 16) |
               ((u32)channel_buf[3] << 24);
    return true;
}

static bool bcdc_get_u32_cmd(u32 cmd, u32 *out)
{
    u8 buf[4] = {0};
    u32 len = sizeof(buf);
    if (!out || !bcdc_get_cmd(cmd, buf, sizeof(buf), &len) || len < 4U)
        return false;
    *out = (u32)buf[0] | ((u32)buf[1] << 8) |
           ((u32)buf[2] << 16) | ((u32)buf[3] << 24);
    return true;
}

static bool bcdc_get_u32_iovar(const char *name, u32 *out)
{
    u8 buf[4] = {0};
    u32 len = sizeof(buf);
    if (!out || !bcdc_get_iovar(name, buf, &len) || len < 4U)
        return false;
    *out = (u32)buf[0] | ((u32)buf[1] << 8) |
           ((u32)buf[2] << 16) | ((u32)buf[3] << 24);
    return true;
}

bool cyw43_join_diag_query(struct cyw_join_diag *out)
{
    if (!out || !cyw43_runtime_ready())
        return false;
    memset(out, 0, sizeof(*out));
    if (bcdc_get_u32_cmd(WLC_GET_RADIO, &out->radio))
        out->valid |= 1U << 0;
    if (bcdc_get_u32_cmd(WLC_GET_CHANNEL, &out->channel))
        out->valid |= 1U << 1;
    if (bcdc_get_u32_cmd(WLC_GET_WSEC, &out->wsec))
        out->valid |= 1U << 2;
    if (bcdc_get_u32_cmd(WLC_GET_WPA_AUTH, &out->wpa_auth))
        out->valid |= 1U << 3;
    if (bcdc_get_u32_iovar("mfp", &out->mfp))
        out->valid |= 1U << 4;
    if (bcdc_get_u32_iovar("sup_wpa", &out->sup_wpa))
        out->valid |= 1U << 5;
    u32 bcmerror = 0U;
    if (bcdc_get_u32_iovar("bcmerror", &bcmerror)) {
        out->bcmerror = (i32)bcmerror;
        out->valid |= 1U << 6;
    }
    u32 bssid_len = sizeof(out->bssid);
    if (bcdc_get_cmd(WLC_GET_BSSID, out->bssid,
                     sizeof(out->bssid), &bssid_len) &&
        bssid_len >= CYW_MAC_LEN)
        out->valid |= 1U << 7;
    return out->valid != 0U;
}

bool cyw43_assoc_req_ies(u8 *out, u32 max, u32 *out_len,
                         u32 *req_reported, u32 *resp_reported,
                         u32 *stage)
{
    if (stage) *stage = 1U;
    if (req_reported) *req_reported = 0U;
    if (resp_reported) *resp_reported = 0U;
    if (!out || !out_len || max == 0U || max > 512U ||
        !cyw43_runtime_ready())
        return false;
    u8 info[512];
    u32 info_len = sizeof(info);
    if (!bcdc_get_iovar("assoc_info", info, &info_len) || info_len < 12U) {
        if (stage) *stage = 2U;
        return false;
    }
    u32 req_len = load_le32(info);
    u32 resp_len = load_le32(info + 4U);
    if (req_reported) *req_reported = req_len;
    if (resp_reported) *resp_reported = resp_len;
    if (req_len == 0U || req_len > max) {
        if (stage) *stage = 3U;
        return false;
    }
    u32 actual = max;
    if (!bcdc_get_iovar("assoc_req_ies", out, &actual)) {
        if (stage) *stage = 4U;
        return false;
    }
    if (actual < req_len) {
        if (stage) *stage = 5U;
        return false;
    }
    *out_len = req_len;
    if (stage) *stage = 6U;
    return true;
}

bool cyw43_set_mac(const u8 mac[CYW_MAC_LEN])
{
    if (!mac || (mac[0] & 1U) != 0U)
        return false;
    if (!bcdc_set_iovar("cur_etheraddr", mac, CYW_MAC_LEN, false))
        return false;
    memcpy(cyw_mac, mac, CYW_MAC_LEN);
    return true;
}

bool cyw43_fwlog(char *out, u32 max, u32 *out_len)
{
    if (!out || !out_len || max == 0U || cyw_ram_bytes < 4U)
        return false;
    u32 shared = 0U;
    if (!bp_read32(cyw_ram_base + cyw_ram_bytes - 4U, &shared) ||
        shared < cyw_ram_base ||
        shared > cyw_ram_base + cyw_ram_bytes - 28U)
        return false;
    u32 console = 0U;
    if (!bp_read32(shared + 20U, &console) ||
        console < cyw_ram_base ||
        console > cyw_ram_base + cyw_ram_bytes - 20U)
        return false;
    u32 log_buf = 0U;
    u32 log_size = 0U;
    u32 log_idx = 0U;
    if (!bp_read32(console + 8U, &log_buf) ||
        !bp_read32(console + 12U, &log_size) ||
        !bp_read32(console + 16U, &log_idx) ||
        log_size == 0U || log_size > 2024U || log_idx > log_size ||
        log_buf < cyw_ram_base ||
        log_buf > cyw_ram_base + cyw_ram_bytes - log_size)
        return false;
    static u8 console_buf[2024] ALIGNED(64);
    if (!bp_read_buf(log_buf, console_buf, log_size))
        return false;
    u32 written = 0U;
    for (u32 i = 0U; i < log_size && written + 1U < max; i++) {
        u8 ch = console_buf[(log_idx + i) % log_size];
        if (ch == 0U)
            continue;
        out[written++] = (ch == '\n' || ch == '\r' ||
                          (ch >= 32U && ch < 127U)) ? (char)ch : '.';
    }
    out[written] = '\0';
    *out_len = written;
    return true;
}

bool cyw43_take_eapol(u8 *frame, u32 *len)
{
    if (!frame || !len || cyw_eapol_len == 0U)
        return false;
    if (*len < cyw_eapol_len)
        return false;
    memcpy(frame, cyw_eapol_frame, cyw_eapol_len);
    *len = cyw_eapol_len;
    cyw_eapol_len = 0U;
    return true;
}

void cyw43_wpa_debug_snapshot(struct cyw_wpa_debug *out)
{
    if (!out)
        return;
    memset(out,0,sizeof(*out));
    out->m1_len=cyw_eapol_len;
    out->m2_len=cyw_m2_len;
    memcpy(out->snonce,wpa_host.snonce,sizeof(out->snonce));
    if (out->m1_len <= sizeof(out->m1))
        memcpy(out->m1,cyw_eapol_frame,out->m1_len);
    if (out->m2_len <= sizeof(out->m2))
        memcpy(out->m2,cyw_m2_frame,out->m2_len);
}

static bool cyw43_join_key(const char *ssid, u32 ssid_len,
                           const u8 *key, u32 key_len, u16 key_flags,
                           u32 security, bool sae)
{
    if (ssid_len == 0 || ssid_len > CYW_SSID_MAX)
        return false;
    if (!key || key_len > CYW_PASSPHRASE_MAX)
        return false;
    scan_kicks_remaining = 0U;
    struct cyw_scan_result fixed_target;
    const struct cyw_scan_result *best = NULL;
    i32 best_rssi = -32768;
    if (join_target.valid) {
        for (u32 i = 0U; i < scan_count; i++) {
            const struct cyw_scan_result *candidate = &scan_results[i];
            if (candidate->ssid_len == ssid_len &&
                memcmp(candidate->ssid,ssid,ssid_len) == 0 &&
                memcmp(candidate->bssid,join_target.bssid,CYW_MAC_LEN) == 0 &&
                candidate->chanspec == join_target.chanspec) {
                best=candidate;
                break;
            }
        }
        if (!best) {
            memset(&fixed_target,0,sizeof(fixed_target));
            fixed_target.ssid_len=(u8)ssid_len;
            memcpy(fixed_target.ssid,ssid,ssid_len);
            memcpy(fixed_target.bssid,join_target.bssid,CYW_MAC_LEN);
            fixed_target.chanspec=join_target.chanspec;
            best=&fixed_target;
        }
        join_target.valid=false;
    } else {
        for (u32 i = 0U; i < scan_count; i++) {
            const struct cyw_scan_result *candidate = &scan_results[i];
            if (candidate->ssid_len != ssid_len ||
                memcmp(candidate->ssid, ssid, ssid_len) != 0 ||
                candidate->rssi <= best_rssi ||
                candidate->chanspec == 0U)
                continue;
            best = candidate;
            best_rssi = candidate->rssi;
        }
    }
    if (!best)
        return false;

    bool host_supplicant =
        !sae && key_len == 32U && key_flags == 0U;
    wpa_host.enabled = false;
    wpa_host.m2_sent = false;

    uart_puts("[cyw] join radio\n");
    if (!cyw43_radio_enable())
        return false;
    cyw_link = CYW_LINK_JOINING;

    static const u8 wpa2_ccmp_psk_rsn[] = {
        0x30U, 0x14U,
        0x01U, 0x00U,
        0x00U, 0x0FU, 0xACU, 0x04U,
        0x01U, 0x00U,
        0x00U, 0x0FU, 0xACU, 0x04U,
        0x01U, 0x00U,
        0x00U, 0x0FU, 0xACU, 0x02U,
        0x00U, 0x00U
    };
    static u8 rsn_buf[sizeof(wpa2_ccmp_psk_rsn)];
    memcpy(rsn_buf, wpa2_ccmp_psk_rsn, sizeof(wpa2_ccmp_psk_rsn));
    if (cyw_rsn_caps_override) {
        rsn_buf[sizeof(rsn_buf)-2U] = (u8)(cyw_rsn_caps & 0xFFU);
        rsn_buf[sizeof(rsn_buf)-1U] = (u8)(cyw_rsn_caps >> 8);
    }
    const u8 *rsn_ie=rsn_buf;
    u32 rsn_ie_len=sizeof(rsn_buf);
    if (!sae && security != WSEC_NONE &&
        !bcdc_set_iovar("wpaie",rsn_ie,rsn_ie_len,false))
        return false;


    u32 wsec;
    u32 wpa_auth;
    if (security == WSEC_NONE) {
        wsec = WSEC_NONE;
        wpa_auth = WPA_AUTH_DISABLED;
    } else {
        wsec = WSEC_AES;
        wpa_auth = sae ? WPA3_AUTH_SAE_PSK : WPA2_AUTH_PSK;
    }

    /* Configure the firmware WPA supplicant. */
    if (security != WSEC_NONE && key_len > 0U) {
        if (host_supplicant) {
            /* Match Circle's external-supplicant sequence. The transitional
             * mode lets the firmware associate before the host completes the
             * four-way handshake. */
            u32 transitional = WPA2_AUTH_PSK | 0x0040U;
            if (!bcdc_set_iovar("wpa_auth",&transitional,
                                sizeof(transitional),false))
                return false;
        } else if (sae) {
            struct {
                u16 key_len;
                u8 key[128];
            } PACKED sae_password;
            memset(&sae_password, 0, sizeof(sae_password));
            sae_password.key_len = (u16)key_len;
            memcpy(sae_password.key, key, key_len);
            if (!bcdc_set_iovar("sae_password", &sae_password,
                                sizeof(sae_password), false))
                return false;
        } else {
            u32 supplicant = 1U;
            uart_puts("[cyw] join supplicant\n");
            if (!bcdc_set_iovar("sup_wpa", &supplicant,
                                sizeof(supplicant), false))
                return false;
            timer_delay_ms(2U);
            struct {
                u16 key_len;
                u16 flags;
                u8  key[CYW_PASSPHRASE_MAX];
            } PACKED wsec_pmk;
            memset(&wsec_pmk, 0, sizeof(wsec_pmk));
            wsec_pmk.key_len = (u16)key_len;
            wsec_pmk.flags = key_flags;
            memcpy(wsec_pmk.key, key, key_len);
            if (!bcdc_set_cmd(WLC_SET_WSEC_PMK, &wsec_pmk,
                              sizeof(wsec_pmk), false))
                return false;
        }
    }

    u32 auth = sae ? 3U : 0U;
    u32 mfp = security == WSEC_NONE ? 0U : 1U;
    uart_puts("[cyw] join auth\n");
    if (!bcdc_set_cmd(WLC_SET_AUTH, &auth, sizeof(auth), false))
        return false;
    uart_puts("[cyw] join wsec\n");
    if (!bcdc_set_cmd(WLC_SET_WSEC, &wsec, sizeof(wsec), false))
        return false;
    if (sae && !bcdc_set_iovar("mfp", &mfp, sizeof(mfp), false))
        return false;
    if (host_supplicant) {
        if (!bcdc_set_iovar("wpa_auth",&wpa_auth,sizeof(wpa_auth),false))
            return false;
    } else if (!bcdc_set_cmd(WLC_SET_WPA_AUTH, &wpa_auth,
                             sizeof(wpa_auth), false)) {
        return false;
    }

    /* WLC_SET_SSID takes brcmf_join_params. Supplying the scanned BSSID and
     * chanspec avoids the firmware's malformed zero-chanspec path on all
     * supported CYW43 board profiles. */
    struct {
        u32 ssid_len;
        u8  ssid[CYW_SSID_MAX];
        u8  bssid[CYW_MAC_LEN];
        u32 chanspec_num;
        u16 chanspec_list[1];
    } join_params;
    _Static_assert(sizeof(join_params) == 52U,
                   "Broadcom join parameters must include association data");
    memset(&join_params, 0, sizeof(join_params));
    join_params.ssid_len = ssid_len;
    memcpy(join_params.ssid, ssid, ssid_len);
    memcpy(join_params.bssid, best->bssid, CYW_MAC_LEN);
    join_params.chanspec_num = 1U;
    join_params.chanspec_list[0] = best->chanspec;

    uart_puts("[cyw] join SSID: ");
    for (u32 i = 0; i < ssid_len; i++)
        uart_putc(ssid[i]);
    uart_puts("\n");

    /* Keep only this association attempt's asynchronous evidence. */
    event_history_reset();
    join_kicks_remaining = CYW_SCAN_KICKS_MAX;
    join_next_kick_ms = timer_monotonic_ms() + CYW_SCAN_KICK_INTERVAL_MS;
    if (!bcdc_set_cmd(WLC_SET_SSID, &join_params,
                      sizeof(join_params), false))
        return false;

    if (host_supplicant) {
        wpa_host.nonce_ready=false;
        memcpy(wpa_host.pmk,key,32U);
        memcpy(wpa_host.ap_mac,best->bssid,CYW_MAC_LEN);
        memcpy(wpa_host.rsn_ie,rsn_ie,rsn_ie_len);
        wpa_host.rsn_ie_len=(u8)rsn_ie_len;
        wpa_host.enabled=true;
    }

    /* Association is asynchronous. The core-0 reactor owns SDPCM/event
     * draining; blocking here used to starve the wired fail-safe path and
     * erase diagnostics in a watchdog reboot. */
    uart_puts("[cyw] joining async\n");
    return true;
}

bool cyw43_join(const char *ssid, u32 ssid_len,
                const char *passphrase, u32 pass_len,
                u32 security)
{
    return cyw43_join_key(ssid, ssid_len, (const u8 *)passphrase,
                          pass_len, 1U, security, false);
}

bool cyw43_join_pmk(const char *ssid, u32 ssid_len, const u8 pmk[32])
{
    return cyw43_join_key(ssid, ssid_len, pmk, 32U, 0U,
                          WSEC_AES, false);
}

bool cyw43_set_join_target(const u8 bssid[CYW_MAC_LEN], u16 chanspec)
{
    if (!bssid || chanspec == 0U || (bssid[0] & 1U) != 0U)
        return false;
    memcpy(join_target.bssid,bssid,CYW_MAC_LEN);
    join_target.chanspec=chanspec;
    join_target.valid=true;
    return true;
}

void cyw43_set_rsn_caps(u16 caps, bool override)
{
    cyw_rsn_caps = caps;
    cyw_rsn_caps_override = override;
}

void cyw43_get_rsn_caps(u16 *caps, bool *override)
{
    if (caps)
        *caps = cyw_rsn_caps_override ? cyw_rsn_caps : 0U;
    if (override)
        *override = cyw_rsn_caps_override;
}

void cyw43_set_rx_probe(bool enabled)
{
    cyw_rx_probe_enabled = enabled;
}

bool cyw43_get_rx_probe(void)
{
    return cyw_rx_probe_enabled;
}

void cyw43_register_sdio_irq(void)
{
    sdio_card_irq_arm();
}

bool cyw43_join_sae(const char *ssid, u32 ssid_len,
                    const char *password, u32 password_len)
{
    return cyw43_join_key(ssid, ssid_len, (const u8 *)password,
                          password_len, 0U, WSEC_AES, true);
}

bool cyw43_disconnect(void)
{
    cyw_link = CYW_LINK_DOWN;
    return bcdc_set_cmd(WLC_DISASSOC, NULL, 0, false);
}

u32 cyw43_link_state(void)
{
    return cyw_link;
}

bool cyw43_is_connected(void)
{
    return cyw_link == CYW_LINK_UP;
}

i32 cyw43_get_rssi(void)
{
    u8 resp[8];
    u32 resp_len = sizeof(resp);
    if (!bcdc_get_iovar("rssi", resp, &resp_len))
        return -127;

    if (resp_len >= 4)
        return (i32)((u32)resp[0] | ((u32)resp[1] << 8) |
                     ((u32)resp[2] << 16) | ((u32)resp[3] << 24));
    return -127;
}

void cyw43_get_mac(u8 *mac)
{
    /* Check if MAC has been retrieved (all zeros = not yet) */
    bool all_zero = true;
    for (u32 i = 0; i < CYW_MAC_LEN; i++) {
        if (cyw_mac[i] != 0) { all_zero = false; break; }
    }
    if (all_zero) {
        /* Try reading from chip */
        u8 resp[8];
        u32 resp_len = sizeof(resp);
        if (bcdc_get_iovar("cur_etheraddr", resp, &resp_len) && resp_len >= 6)
            memcpy(cyw_mac, resp, 6);
    }
    memcpy(mac, cyw_mac, CYW_MAC_LEN);
}

bool cyw43_send_frame(const u8 *frame, u32 len)
{
    bool eapol = frame && len >= 14U &&
                 frame[12] == 0x88U && frame[13] == 0x8EU;
    if (!cyw43_is_connected() &&
        !(cyw_link == CYW_LINK_JOINING && eapol))
        return false;

    /* Prepend 4-byte BDC data header before the Ethernet frame */
    u32 bdc_len = 4 + len;
    if (bdc_len > sizeof(cyw_tx_buf) - SDPCM_HEADER_LEN)
        return false;

    static u8 bdc_buf[CYW_MAX_FRAME] ALIGNED(64);
    bdc_buf[0] = 0x20;  /* BDC version 2 */
    bdc_buf[1] = 0x00;  /* 802.1D priority */
    bdc_buf[2] = 0x00;  /* header2 */
    bdc_buf[3] = 0x00;  /* pad */
    memcpy(bdc_buf + 4, frame, len);

    return sdpcm_send(SDPCM_DATA_CHANNEL, bdc_buf, bdc_len);
}

bool cyw43_recv_frame(u8 *frame, u32 *len)
{
    u8 channel;
    u32 rlen;

    if (cyw_data_rx_count != 0U) {
        struct cyw_data_rx_slot *slot =
            &cyw_data_rx_queue[cyw_data_rx_tail];
        bool ok = cyw_data_frame_copy(slot->data, slot->len, frame, len);
        cyw_data_rx_tail =
            (cyw_data_rx_tail + 1U) % CYW_DATA_RX_QUEUE_DEPTH;
        cyw_data_rx_count--;
        return ok;
    }

    if (!sdpcm_recv(&channel, cyw_rx_buf, &rlen))
        return false;

    if (channel == SDPCM_EVENT_CHANNEL) {
        handle_event(cyw_rx_buf, rlen);
        return false;
    }

    if (channel != SDPCM_DATA_CHANNEL)
        return false;

    return cyw_data_frame_copy(cyw_rx_buf, rlen, frame, len);
}
