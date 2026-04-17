/*
 * cyw43.c - CYW43455 WiFi FullMAC driver
 *
 * Broadcom/Cypress CYW43455 combo chip on Pi 5, via SDIO over RP1.
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
#include "uart.h"
#include "timer.h"
#include "fb.h"

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

/* RESETCTRL bits */
#define AIRC_RESET              0x0001

/* SOCSRAM wrapper */
#define SOCSRAM_BANKX_IDX       0x10
#define SOCSRAM_BANKX_PDA       0x44

/* Clock control status register (SDIO func 1) */
#define SDIO_CLKCSR             0x1000E
#define SDIO_PULLUPS            0x1000F
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
#define SDIOD_SBMBOXDATA        0x48

/* Firmware upload block size */
#define FW_UPLOAD_BLKSZ         64
#define SDIO_FUNC1_BLKSZ        64
#define SDIO_FUNC2_BLKSZ        512

/* SDPCM frame tags */
#define SDPCM_FRAMETAG_LEN_MASK 0xFFFF

/* Maximum frame size */
#define CYW_MAX_FRAME           2048

/* ── State ── */

static u8 cyw_mac[CYW_MAC_LEN];
static u32 cyw_link;
static u8 cyw_tx_seq;
static u32 cyw_backplane_window;

/* RX/TX buffers — 64-byte aligned for SDIO DMA compatibility */
static u8 cyw_tx_buf[CYW_MAX_FRAME] ALIGNED(64);
static u8 cyw_rx_buf[CYW_MAX_FRAME] ALIGNED(64);

/* BCDC request ID counter */
static u16 bcdc_reqid;

/* Scan results */
static struct cyw_scan_result scan_results[CYW_MAX_SCAN_RESULTS];
static u32 scan_count;
static bool scan_in_progress;

/* Discovered core addresses (from EROM scan) */
static u32 cyw_arm_ctl;
static u32 cyw_d11_ctl;
static u32 cyw_sram_ctl;
static u32 cyw_sdio_regs;

/* ── Backplane access ── */

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

/* CMD52-based 4-byte backplane write — avoids DAT line entirely */
static bool bp_write32(u32 addr, u32 val)
{
    if (!bp_set_window(addr)) {
        uart_puts("[bpw] win fail @");
        uart_hex(addr);
        uart_puts("\n");
        return false;
    }

    u32 off = addr & BACKPLANE_WIN_MASK;
    if (!sdio_cmd52_write(SDIO_FUNC_BACKPLANE, (off | 0x8000) + 0, (u8)(val & 0xFF))) {
        uart_puts("[bpw] w52 fail off=");
        uart_hex(off);
        uart_puts("\n");
        return false;
    }
    if (!sdio_cmd52_write(SDIO_FUNC_BACKPLANE, (off | 0x8000) + 1, (u8)((val >> 8) & 0xFF))) return false;
    if (!sdio_cmd52_write(SDIO_FUNC_BACKPLANE, (off | 0x8000) + 2, (u8)((val >> 16) & 0xFF))) return false;
    if (!sdio_cmd52_write(SDIO_FUNC_BACKPLANE, (off | 0x8000) + 3, (u8)((val >> 24) & 0xFF))) return false;
    return true;
}

UNUSED static bool bp_read_buf(u32 addr, u8 *buf, u32 len)
{
    while (len > 0) {
        if (!bp_set_window(addr))
            return false;

        u32 off = addr & BACKPLANE_WIN_MASK;
        u32 chunk = BACKPLANE_WIN_SIZE - off;
        if (chunk > len) chunk = len;
        if (chunk > 512) chunk = 512;

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
    while (len > 0) {
        if (!bp_set_window(addr))
            return false;

        u32 off = addr & BACKPLANE_WIN_MASK;
        u32 chunk = BACKPLANE_WIN_SIZE - off;
        if (chunk > len) chunk = len;
        if (chunk > 512) chunk = 512;

        if (!sdio_cmd53_write(SDIO_FUNC_BACKPLANE, off | 0x8000,
                              buf, chunk, true))
            return false;

        addr += chunk;
        buf += chunk;
        len -= chunk;
    }
    return true;
}

/* ── Core control ── */

static bool core_disable(u32 core_base)
{
    u32 val;

    /* Check if already in reset */
    if (!bp_read32(core_base + CORE_RESETCTRL, &val))
        return false;
    if (val & AIRC_RESET)
        return true;

    /* Disable clocks */
    if (!bp_write32(core_base + CORE_IOCTRL, 0))
        return false;
    if (!bp_read32(core_base + CORE_IOCTRL, &val))
        return false;
    delay_cycles(1000);

    /* Put core in reset */
    if (!bp_write32(core_base + CORE_RESETCTRL, AIRC_RESET))
        return false;
    delay_cycles(1000);

    return true;
}

static bool core_reset(u32 core_base, u32 ioctrl_flags)
{
    /* Disable first */
    if (!core_disable(core_base))
        return false;

    /* Pull out of reset with clocks + flags */
    if (!bp_write32(core_base + CORE_IOCTRL,
                    SICF_FGC | SICF_CLOCK_EN | ioctrl_flags))
        return false;
    u32 val;
    if (!bp_read32(core_base + CORE_IOCTRL, &val))
        return false;
    delay_cycles(1000);

    if (!bp_write32(core_base + CORE_RESETCTRL, 0))
        return false;
    delay_cycles(1000);

    /* Remove force-gated clocks */
    if (!bp_write32(core_base + CORE_IOCTRL,
                    SICF_CLOCK_EN | ioctrl_flags))
        return false;
    if (!bp_read32(core_base + CORE_IOCTRL, &val))
        return false;
    delay_cycles(1000);

    return true;
}

/* ── Chip identification ── */

static bool chip_identify(void)
{
    u32 chip_id;
    if (!bp_read32(CYW_CHIPCOMMON_BASE, &chip_id))
        return false;

    u16 id = (u16)(chip_id & 0xFFFF);
    u16 rev = (u16)((chip_id >> 16) & 0xF);

    uart_puts("[cyw] chip=");
    uart_hex(id);
    uart_puts(" r=");
    uart_hex(rev);
    uart_puts("\n");

    if (id != CYW43455_CHIP_ID) {
        uart_puts("[cyw] bad chip ID\n");
        return false;
    }
    return true;
}

/* ── SDPCM / BCDC ── */

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

static bool sdpcm_send(u8 channel, const u8 *data, u32 len)
{
    if (len + SDPCM_HEADER_LEN > CYW_MAX_FRAME)
        return false;

    u32 hdr_len = sdpcm_build_header(cyw_tx_buf, len, channel);
    memcpy(cyw_tx_buf + hdr_len, data, len);

    u32 total = hdr_len + len;
    /* Pad to 64-byte boundary for SDIO */
    u32 padded = (total + 63) & ~63U;

    /* Zero padding to avoid leaking stale buffer contents */
    if (padded > total)
        memset(cyw_tx_buf + total, 0, padded - total);

    /* Use block mode for transfers >512 bytes */
    if (padded > 512) {
        u32 nblks = (padded + SDIO_FUNC2_BLKSZ - 1) / SDIO_FUNC2_BLKSZ;
        return sdio_cmd53_write_blocks(SDIO_FUNC_WLAN, 0, cyw_tx_buf,
                                       SDIO_FUNC2_BLKSZ, nblks, true);
    }
    return sdio_cmd53_write(SDIO_FUNC_WLAN, 0, cyw_tx_buf, padded, true);
}

static bool sdpcm_recv(u8 *channel, u8 *data, u32 *len)
{
    /* Read via block mode — frames can exceed 512 bytes */
    u32 nblks = (CYW_MAX_FRAME + SDIO_FUNC2_BLKSZ - 1) / SDIO_FUNC2_BLKSZ;
    if (!sdio_cmd53_read_blocks(SDIO_FUNC_WLAN, 0, cyw_rx_buf,
                                SDIO_FUNC2_BLKSZ, nblks, true))
        return false;

    /* Parse frame tag */
    u16 frame_len = (u16)cyw_rx_buf[0] | ((u16)cyw_rx_buf[1] << 8);
    u16 frame_not = (u16)cyw_rx_buf[2] | ((u16)cyw_rx_buf[3] << 8);

    if (frame_len == 0 || frame_len > CYW_MAX_FRAME)
        return false;
    if ((u16)(frame_len ^ frame_not) != 0xFFFF)
        return false;

    *channel = cyw_rx_buf[5] & 0x0F;
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

static bool bcdc_set_iovar(const char *name, const void *data, u32 data_len)
{
    u32 name_len = pios_strlen(name) + 1;
    u32 body_len = name_len + data_len;
    u32 payload_len = BCDC_HEADER_LEN + body_len;

    if (payload_len > CYW_MAX_FRAME - SDPCM_HEADER_LEN)
        return false;

    static u8 buf[CYW_MAX_FRAME] ALIGNED(64);
    if (payload_len > sizeof(buf))
        return false;
    u16 id = bcdc_reqid++;

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

    return sdpcm_send(SDPCM_CTL_CHANNEL, buf, payload_len);
}

static bool bcdc_get_iovar(const char *name, u8 *resp, u32 *resp_len)
{
    u32 name_len = pios_strlen(name) + 1;
    u32 payload_len = BCDC_HEADER_LEN + name_len;

    u8 buf[512];
    u16 id = bcdc_reqid++;

    /* BCDC 16-byte header (little-endian) */
    u32 cmd = WLC_GET_VAR;
    buf[0]  = (u8)(cmd & 0xFF);
    buf[1]  = (u8)((cmd >> 8) & 0xFF);
    buf[2]  = (u8)((cmd >> 16) & 0xFF);
    buf[3]  = (u8)((cmd >> 24) & 0xFF);
    buf[4]  = (u8)(name_len & 0xFF);
    buf[5]  = (u8)((name_len >> 8) & 0xFF);
    buf[6]  = (u8)((name_len >> 16) & 0xFF);
    buf[7]  = (u8)((name_len >> 24) & 0xFF);
    buf[8]  = 0; buf[9] = 0;   /* flags = 0 (GET) */
    buf[10] = (u8)(id & 0xFF);
    buf[11] = (u8)((id >> 8) & 0xFF);
    buf[12] = 0; buf[13] = 0; buf[14] = 0; buf[15] = 0; /* status = 0 */

    /* iovar name */
    memcpy(buf + BCDC_HEADER_LEN, name, name_len);

    if (!sdpcm_send(SDPCM_CTL_CHANNEL, buf, payload_len))
        return false;

    /* Wait for response */
    u32 timeout = 100;
    while (timeout--) {
        u8 channel;
        u32 rlen;
        if (sdpcm_recv(&channel, cyw_rx_buf, &rlen)) {
            if (channel == SDPCM_CTL_CHANNEL && rlen >= BCDC_HEADER_LEN) {
                /* Check status field at offset 12 */
                u32 status = (u32)cyw_rx_buf[12] | ((u32)cyw_rx_buf[13] << 8) |
                             ((u32)cyw_rx_buf[14] << 16) | ((u32)cyw_rx_buf[15] << 24);
                if (status != 0) {
                    uart_puts("[cyw] bcdc get err st=");
                    uart_hex(status);
                    uart_puts("\n");
                    return false;
                }
                u32 data_off = BCDC_HEADER_LEN;
                u32 data_len = rlen - data_off;
                if (resp && data_len > 0) {
                    if (data_len > *resp_len)
                        data_len = *resp_len;
                    memcpy(resp, cyw_rx_buf + data_off, data_len);
                }
                *resp_len = data_len;
                return true;
            }
        }
        delay_cycles(10000);
    }
    return false;
}

static bool bcdc_set_cmd(u32 cmd, const void *data, u32 data_len)
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

    return sdpcm_send(SDPCM_CTL_CHANNEL, buf, payload_len);
}

UNUSED static bool bcdc_get_cmd(u32 cmd, u8 *resp, u32 *resp_len)
{
    u32 payload_len = BCDC_HEADER_LEN;

    u8 buf[512];
    u16 id = bcdc_reqid++;

    /* BCDC 16-byte header (little-endian) */
    buf[0]  = (u8)(cmd & 0xFF);
    buf[1]  = (u8)((cmd >> 8) & 0xFF);
    buf[2]  = (u8)((cmd >> 16) & 0xFF);
    buf[3]  = (u8)((cmd >> 24) & 0xFF);
    buf[4]  = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0; /* len = 0 */
    buf[8]  = 0; buf[9] = 0;   /* flags = 0 (GET) */
    buf[10] = (u8)(id & 0xFF);
    buf[11] = (u8)((id >> 8) & 0xFF);
    buf[12] = 0; buf[13] = 0; buf[14] = 0; buf[15] = 0; /* status = 0 */

    if (!sdpcm_send(SDPCM_CTL_CHANNEL, buf, payload_len))
        return false;

    u32 timeout = 100;
    while (timeout--) {
        u8 channel;
        u32 rlen;
        if (sdpcm_recv(&channel, cyw_rx_buf, &rlen)) {
            if (channel == SDPCM_CTL_CHANNEL && rlen >= BCDC_HEADER_LEN) {
                /* Check status field at offset 12 */
                u32 status = (u32)cyw_rx_buf[12] | ((u32)cyw_rx_buf[13] << 8) |
                             ((u32)cyw_rx_buf[14] << 16) | ((u32)cyw_rx_buf[15] << 24);
                if (status != 0) {
                    uart_puts("[cyw] bcdc get err st=");
                    uart_hex(status);
                    uart_puts("\n");
                    return false;
                }
                u32 data_off = BCDC_HEADER_LEN;
                u32 data_len = rlen - data_off;
                if (resp && data_len > 0) {
                    if (data_len > *resp_len)
                        data_len = *resp_len;
                    memcpy(resp, cyw_rx_buf + data_off, data_len);
                }
                *resp_len = data_len;
                return true;
            }
        }
        delay_cycles(10000);
    }
    return false;
}

/* ── Event handling ── */

static void handle_event(const u8 *data, u32 len)
{
    if (len < 48)
        return;

    /* Event message starts after Ethernet header (14 bytes)
     * + Broadcom OUI header (10 bytes) = 24 bytes.
     * Event type at offset 24+4 = 28 (big-endian u32) */
    u32 event_type = ((u32)data[28] << 24) | ((u32)data[29] << 16) |
                     ((u32)data[30] << 8) | (u32)data[31];
    u32 status = ((u32)data[36] << 24) | ((u32)data[37] << 16) |
                 ((u32)data[38] << 8) | (u32)data[39];

    switch (event_type) {
    case CYW_E_SET_SSID:
        if (status == CYW_E_STATUS_SUCCESS) {
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
            /* Check flags at offset 44 for link up/down */
            u16 flags = ((u16)data[44] << 8) | (u16)data[45];
            if (flags & 1) {
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
        if (status == CYW_E_STATUS_PARTIAL && len >= 80) {
            /* Parse BSS info from escan result */
            if (scan_count < CYW_MAX_SCAN_RESULTS) {
                struct cyw_scan_result *r = &scan_results[scan_count];
                /* BSSID at offset 48 */
                memcpy(r->bssid, data + 48, CYW_MAC_LEN);
                /* RSSI at offset 54 (i16 LE) */
                r->rssi = (i16)((u16)data[54] | ((u16)data[55] << 8));
                /* Channel at offset 58 */
                r->channel = data[58];
                /* SSID length at offset 70, bounded by buffer */
                r->ssid_len = data[70];
                if (r->ssid_len > CYW_SSID_MAX)
                    r->ssid_len = CYW_SSID_MAX;
                if (71 + r->ssid_len > len)
                    r->ssid_len = (len > 71) ? (u8)(len - 71) : 0;
                memcpy(r->ssid, data + 71, r->ssid_len);
                scan_count++;
            }
        } else if (status == CYW_E_STATUS_SUCCESS) {
            scan_in_progress = false;
            uart_puts("[cyw] scan done n=");
            uart_hex(scan_count);
            uart_puts("\n");
        }
        break;

    default:
        break;
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
    u32 ram_size = 0x80000; /* TODO: read from SOCSRAM sizing regs */

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

    if (!bp_write_buf(CYW_RAM_BASE + nvram_offset, nvram_condensed, clen)) {
        uart_puts("[cyw] NVRAM write fail\n");
        return false;
    }

    /* Write length token: complement of size-in-words in upper 16 bits */
    u32 token = (~(clen / 4) << 16) | (clen / 4);
    if (!bp_write32(CYW_RAM_BASE + ram_size - 4, token)) {
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
    u32 arm_ctl = 0, d11_ctl = 0, sram_ctl = 0, sdio_regs = 0;
    u32 coreid = 0;
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
                coreid = ((entry >> 4) | ((entry >> 8) & 0xFF00)) & 0xFFF;
                i += 4;
            }
        } else if (tag == 0x5) {  /* address descriptor */
            u32 addr = ((entry >> 8) & 0xFFFFF0) << 8;
            addr &= ~0xFFFU;
            bool is_ctl = (entry & 0xC0) != 0;
            switch (coreid) {
            case 0x83C: case 0x83E:  /* ARM CR4 / CA7 */
                if (is_ctl && !arm_ctl) arm_ctl = addr;
                break;
            case 0x80E:  /* SOCSRAM */
                if (is_ctl && !sram_ctl) sram_ctl = addr;
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
    uart_puts(" D11=");
    uart_hex(d11_ctl);
    uart_puts(" SRAM=");
    uart_hex(sram_ctl);
    uart_puts(" SDIO=");
    uart_hex(sdio_regs);
    uart_puts("\n");

    if (!arm_ctl || !sram_ctl) {
        uart_puts("[cyw] core scan fail\n");
        return false;
    }

    /* Store discovered addresses for firmware load */
    cyw_arm_ctl = arm_ctl;
    cyw_d11_ctl = d11_ctl;
    cyw_sram_ctl = sram_ctl;
    cyw_sdio_regs = sdio_regs;

    /* Halt ARM CR4 core */
    if (!core_reset(cyw_arm_ctl, SICF_CPUHALT)) {
        uart_puts("[cyw] ARM!\n");
        return false;
    }

    /* Reset D11 802.11 MAC core */
    if (cyw_d11_ctl && !core_reset(cyw_d11_ctl, 4)) {
        uart_puts("[cyw] D11!\n");
        return false;
    }

    /* Reset SOCSRAM, disable remap */
    if (!core_reset(cyw_sram_ctl, 0)) {
        uart_puts("[cyw] SRAM!\n");
        return false;
    }
    bp_write32(cyw_sram_ctl + SOCSRAM_BANKX_IDX, 0x03);
    bp_write32(cyw_sram_ctl + SOCSRAM_BANKX_PDA, 0);

    /* Clear pull-ups/pull-downs */
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_PULLUPS, 0);
    bp_write32(CYW_CHIPCOMMON_BASE + CC_GPIOPULLUP, 0);
    bp_write32(CYW_CHIPCOMMON_BASE + CC_GPIOPULLDOWN, 0);

    return true;
}

/* ── Public API ── */

bool cyw43_init(void)
{
    cyw_link = CYW_LINK_DOWN;
    cyw_tx_seq = 0;
    cyw_backplane_window = 0;
    bcdc_reqid = 1;
    scan_count = 0;
    scan_in_progress = false;
    memset(cyw_mac, 0, CYW_MAC_LEN);

    uart_puts("[cyw] init...\n");

    /* Initialize SDIO controller and enumerate card */
    if (!sdio_init()) {
        uart_puts("[cyw] SDIO fail\n");
        return false;
    }

    /* Enable backplane function (func 1) */
    if (!sdio_enable_func(SDIO_FUNC_BACKPLANE)) {
        uart_puts("[cyw] f1 enable fail\n");
        return false;
    }
    sdio_set_block_size(SDIO_FUNC_BACKPLANE, SDIO_FUNC1_BLKSZ);

    /* Enable 4-bit SDIO bus */
    sdio_set_bus_width_4bit();

    /* Identify chip via backplane */
    if (!chip_identify()) {
        uart_puts("[cyw] chip ID fail\n");
        return false;
    }

    /* Backplane init: halt ARM, reset cores, setup clocks */
    if (!cyw43_backplane_init()) {
        uart_puts("[cyw] bp fail\n");
        return false;
    }

    /* Enable function interrupts for backplane */
    sdio_enable_func_irq(SDIO_FUNC_BACKPLANE);

    /* NOTE: func 2 (WLAN) is enabled AFTER firmware load in cyw43_load_firmware(),
     * because the WLAN function won't be ready until firmware boots. */

    uart_puts("[cyw] init OK\n");
    return true;
}

bool cyw43_load_firmware(void)
{
    /*
     * Load firmware, NVRAM, and CLM blobs from the FAT32 boot partition.
     * Expected files (long filenames supported):
     *   /firmware.bin     (CYW43455 WiFi firmware)
     *   /nvram.txt        (board NVRAM configuration)
     *   /clm.bin          (regulatory/CLM blob)
     */

    /* Initialize FAT32 if not already done */
    if (!fat32_init()) {
        uart_puts("[cyw] FAT32 fail\n");
        return false;
    }

    /* Request ALP clock for backplane memory access */
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, CLKCSR_ReqALP);
    for (u32 i = 0; i < 1000; i++) {
        u8 clk;
        if (sdio_cmd52_read(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, &clk) &&
            (clk & CLKCSR_ALPavail))
            break;
        delay_cycles(5000);
    }

    /* Clear end of RAM before upload */
    bp_write32(CYW_RAM_BASE + CYW_RAM_SIZE - 4, 0);

    /* Upload firmware binary */
    {
        fat32_file_t fw;
        if (!fat32_open("/wifi/firmware.bin", &fw)) {
            uart_puts("[cyw] no fw\n");
            return false;
        }
        uart_puts("[cyw] fw (");
        uart_hex(fw.file_size);
        uart_puts(" bytes)...\n");

        u32 offset = 0;
        static u8 ALIGNED(64) fw_chunk[FW_UPLOAD_BLKSZ];
        while (offset < fw.file_size) {
            u32 chunk = fw.file_size - offset;
            if (chunk > FW_UPLOAD_BLKSZ) chunk = FW_UPLOAD_BLKSZ;
            u32 got = fat32_read(&fw, fw_chunk, chunk);
            if (got == 0) {
                uart_puts("[cyw] fw read err @");
                uart_hex(offset);
                uart_puts("\n");
                fat32_close(&fw);
                return false;
            }
            if (!bp_write_buf(CYW_RAM_BASE + offset, fw_chunk, got)) {
                uart_puts("[cyw] fw write err @");
                uart_hex(offset);
                uart_puts("\n");
                fat32_close(&fw);
                return false;
            }
            offset += got;
        }
        fat32_close(&fw);
        uart_puts("[cyw] fw uploaded\n");
    }

    /* Upload NVRAM */
    {
        fat32_file_t nv;
        if (!fat32_open("/wifi/nvram.txt", &nv)) {
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
        } else {
            static u8 ALIGNED(4) nvram_data[4096];
            u32 nvram_len = fat32_read(&nv, nvram_data, sizeof(nvram_data));
            fat32_close(&nv);
            uart_puts("[cyw] nvram (");
            uart_hex(nvram_len);
            uart_puts(" B)\n");
            if (!upload_nvram(nvram_data, nvram_len)) {
                uart_puts("[cyw] nvram fail\n");
                return false;
            }
        }
    }

    /* Clear SDIOD interrupt status before ARM reset */
    bp_write32(cyw_sdio_regs + SDIOD_INTSTATUS, 0xFFFFFFFF);

    /* Reset ARM core to start firmware */
    if (!core_reset(cyw_arm_ctl, 0)) {
        uart_puts("[cyw] ARM reset fail\n");
        return false;
    }

    /* sbenable: request HT clock and wait (Issue #67) */
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, 0);
    delay_cycles(500000);
    sdio_cmd52_write(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, CLKCSR_ReqHT);
    bool ht_ok = false;
    for (u32 i = 0; i < 50; i++) {
        u8 clk;
        if (sdio_cmd52_read(SDIO_FUNC_BACKPLANE, SDIO_CLKCSR, &clk) &&
            (clk & CLKCSR_HTavail)) {
            ht_ok = true;
            break;
        }
        delay_cycles(5000000);
    }
    if (!ht_ok) {
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
    bp_write32(cyw_sdio_regs + SDIOD_SBMBOXDATA, 4 << 16);

    /* Set interrupt mask: Fcchange|FrameInt|MailboxInt */
    bp_write32(cyw_sdio_regs + SDIOD_INTMASK, 0xE0);

    /* Enable WLAN data function (func 2) — now that firmware is running */
    if (!sdio_enable_func(SDIO_FUNC_WLAN)) {
        uart_puts("[cyw] f2 fail\n");
        return false;
    }
    sdio_set_block_size(SDIO_FUNC_WLAN, SDIO_FUNC2_BLKSZ);
    sdio_enable_func_irq(SDIO_FUNC_WLAN);

    /* Load CLM blob via 'clmload' iovar */
    {
        fat32_file_t clm;
        if (fat32_open("/wifi/clm.bin", &clm)) {
            uart_puts("[cyw] loading CLM (");
            uart_hex(clm.file_size);
            uart_puts(" bytes)...\n");

            static u8 ALIGNED(4) clm_chunk[1024 + 16]; /* data + download header */
            u32 offset = 0;
            bool clm_ok = true;

            while (offset < clm.file_size && clm_ok) {
                u32 chunk = clm.file_size - offset;
                if (chunk > 1024) chunk = 1024;

                /* Build download header: flag(2) + type(2) + len(4) + crc(4) = 12 bytes */
                u16 flag = 0x0004; /* DL_CONT */
                if (offset == 0) flag |= 0x0002; /* DL_BEGIN */
                if (offset + chunk >= clm.file_size) flag |= 0x0008; /* DL_END */

                clm_chunk[0] = flag & 0xFF;
                clm_chunk[1] = (flag >> 8) & 0xFF;
                clm_chunk[2] = 0x02; /* type = CLM */
                clm_chunk[3] = 0x00;
                clm_chunk[4] = chunk & 0xFF;
                clm_chunk[5] = (chunk >> 8) & 0xFF;
                clm_chunk[6] = (chunk >> 16) & 0xFF;
                clm_chunk[7] = (chunk >> 24) & 0xFF;
                clm_chunk[8] = 0; clm_chunk[9] = 0; /* crc = 0 */
                clm_chunk[10] = 0; clm_chunk[11] = 0;

                u32 got = fat32_read(&clm, clm_chunk + 12, chunk);
                if (got != chunk) { clm_ok = false; break; }

                if (!bcdc_set_iovar("clmload", clm_chunk, 12 + chunk))
                    clm_ok = false;

                offset += chunk;
            }
            fat32_close(&clm);
            if (clm_ok) uart_puts("[cyw] CLM ok\n");
            else uart_puts("[cyw] CLM fail\n");
        } else {
            uart_puts("[cyw] no CLM\n");
        }
    }

    uart_puts("[cyw] fw ok\n");
    return true;
}

void cyw43_poll(void)
{
    /* Check for pending data on SDIO func 2 */
    u8 int_pending;
    if (!sdio_cmd52_read(SDIO_FUNC_CIA, CCCR_INT_PENDING, &int_pending))
        return;

    if (!(int_pending & (1 << SDIO_FUNC_WLAN)))
        return;

    /* Read a frame from func 2 */
    u8 channel;
    u32 len;
    if (!sdpcm_recv(&channel, cyw_rx_buf, &len))
        return;

    switch (channel) {
    case SDPCM_EVENT_CHANNEL:
        handle_event(cyw_rx_buf, len);
        break;

    case SDPCM_DATA_CHANNEL:
        /* Data frame ready for wifi_nic to pick up */
        break;

    case SDPCM_CTL_CHANNEL:
        /* Control response — handled inline by bcdc_get_* */
        break;

    default:
        break;
    }
}

bool cyw43_scan_start(void)
{
    scan_count = 0;
    scan_in_progress = true;

    /* Enable escan events */
    u8 evmask[32];
    memset(evmask, 0, sizeof(evmask));
    evmask[CYW_E_ESCAN_RESULT / 8] |= (1 << (CYW_E_ESCAN_RESULT % 8));
    bcdc_set_iovar("event_msgs", evmask, sizeof(evmask));

    /* Start escan */
    struct {
        u32 version;
        u32 action;
        u32 sync_id;
        u32 ssid_len;
        u8  ssid[CYW_SSID_MAX];
        u8  bssid[CYW_MAC_LEN];
        u8  bss_type;
        u8  scan_type;
        u32 nprobes;
        u32 active_time;
        u32 passive_time;
        u32 home_time;
        u32 channel_num;
    } PACKED escan_params;

    memset(&escan_params, 0, sizeof(escan_params));
    escan_params.version = 1;
    escan_params.action = 1;  /* start */
    escan_params.sync_id = 0x1234;
    escan_params.bss_type = 2;  /* any */
    memset(escan_params.bssid, 0xFF, CYW_MAC_LEN);
    escan_params.nprobes = 2;
    escan_params.active_time = 40;
    escan_params.passive_time = 130;
    escan_params.home_time = 45;

    return bcdc_set_iovar("escan", &escan_params, sizeof(escan_params));
}

bool cyw43_scan_get_results(struct cyw_scan_result *results, u32 *count)
{
    if (scan_in_progress)
        return false;

    u32 n = scan_count;
    if (n > *count) n = *count;

    memcpy(results, scan_results, n * sizeof(struct cyw_scan_result));
    *count = n;
    return true;
}

bool cyw43_join(const char *ssid, u32 ssid_len,
                const char *passphrase, u32 pass_len,
                u32 security)
{
    if (ssid_len == 0 || ssid_len > CYW_SSID_MAX)
        return false;
    if (pass_len > CYW_PASSPHRASE_MAX)
        return false;

    cyw_link = CYW_LINK_JOINING;

    /* Set infrastructure mode */
    u32 infra = 1;
    bcdc_set_cmd(WLC_SET_INFRA, &infra, 4);

    /* Set auth mode (open) */
    u32 auth = 0;
    bcdc_set_cmd(WLC_SET_AUTH, &auth, 4);

    /* Set security type */
    u32 wsec;
    u32 wpa_auth;
    if (security == WSEC_NONE) {
        wsec = WSEC_NONE;
        wpa_auth = WPA_AUTH_DISABLED;
    } else {
        wsec = WSEC_AES;
        wpa_auth = WPA2_AUTH_PSK;
    }
    bcdc_set_cmd(WLC_SET_WSEC, &wsec, 4);
    bcdc_set_cmd(WLC_SET_WPA_AUTH, &wpa_auth, 4);

    /* Set passphrase via iovar */
    if (security != WSEC_NONE && pass_len > 0) {
        struct {
            u16 key_len;
            u16 flags;
            u8  key[CYW_PASSPHRASE_MAX];
        } PACKED wsec_pmk;

        memset(&wsec_pmk, 0, sizeof(wsec_pmk));
        wsec_pmk.key_len = (u16)pass_len;
        wsec_pmk.flags = 1;  /* WSEC_PASSPHRASE */
        memcpy(wsec_pmk.key, passphrase, pass_len);

        bcdc_set_iovar("wsec_pmk_info", &wsec_pmk, sizeof(wsec_pmk));
    }

    /* Set SSID to trigger join */
    struct {
        u32 ssid_len;
        u8  ssid[CYW_SSID_MAX];
    } PACKED ssid_params;

    memset(&ssid_params, 0, sizeof(ssid_params));
    ssid_params.ssid_len = ssid_len;
    memcpy(ssid_params.ssid, ssid, ssid_len);

    uart_puts("[cyw] join SSID: ");
    for (u32 i = 0; i < ssid_len; i++)
        uart_putc(ssid[i]);
    uart_puts("\n");

    if (!bcdc_set_cmd(WLC_SET_SSID, &ssid_params, sizeof(ssid_params)))
        return false;

    /* Poll for association result */
    uart_puts("[cyw] joining...\n");
    for (u32 i = 0; i < 200; i++) {  /* up to ~10s */
        cyw43_poll();
        u32 state = cyw43_link_state();
        if (state == CYW_LINK_UP) {
            uart_puts("[cyw] associated!\n");
            return true;
        }
        if (state == CYW_LINK_AUTH_FAIL) {
            uart_puts("[cyw] auth failed\n");
            return false;
        }
        for (volatile u32 d = 0; d < 500000; d++) {} /* ~50ms */
    }
    uart_puts("[cyw] join timeout\n");
    return false;
}

bool cyw43_disconnect(void)
{
    cyw_link = CYW_LINK_DOWN;
    return bcdc_set_cmd(WLC_DISASSOC, NULL, 0);
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
    if (!cyw43_is_connected())
        return false;

    /* Prepend 4-byte BDC data header before the Ethernet frame */
    u32 bdc_len = 4 + len;
    if (bdc_len > sizeof(cyw_tx_buf) - SDPCM_HEADER_LEN)
        return false;

    static u8 bdc_buf[CYW_MAX_FRAME] ALIGNED(64);
    bdc_buf[0] = 0x20;  /* BDC version 2 */
    bdc_buf[1] = 0x00;  /* flags */
    bdc_buf[2] = 0x00;  /* header2 */
    bdc_buf[3] = 0x00;  /* pad */
    memcpy(bdc_buf + 4, frame, len);

    return sdpcm_send(SDPCM_DATA_CHANNEL, bdc_buf, bdc_len);
}

bool cyw43_recv_frame(u8 *frame, u32 *len)
{
    u8 channel;
    u32 rlen;

    if (!sdpcm_recv(&channel, cyw_rx_buf, &rlen))
        return false;

    if (channel == SDPCM_EVENT_CHANNEL) {
        handle_event(cyw_rx_buf, rlen);
        return false;
    }

    if (channel != SDPCM_DATA_CHANNEL)
        return false;

    /* Strip BDC data header: 4 bytes + (data[3] * 4) bytes of padding */
    if (rlen < 4)
        return false;
    u32 bdc_offset = 4 + ((u32)cyw_rx_buf[3] << 2);
    if (bdc_offset >= rlen)
        return false;
    rlen -= bdc_offset;

    if (rlen > *len)
        rlen = *len;

    memcpy(frame, cyw_rx_buf + bdc_offset, rlen);
    *len = rlen;
    return true;
}
