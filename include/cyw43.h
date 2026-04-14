/*
 * cyw43.h - CYW43455 WiFi FullMAC driver
 *
 * The CYW43455 is a Broadcom/Cypress combo WiFi+BT chip on the Pi 5,
 * connected via SDIO through the RP1 southbridge. In FullMAC mode,
 * the chip firmware handles 802.11 MAC and WPA2/WPA3 internally.
 * The host communicates via Broadcom SDPCM/BCDC protocol.
 *
 * Phases:
 *   1. Chip init (power, SDIO enumeration, backplane access)
 *   2. Firmware boot (upload firmware+NVRAM+CLM to chip RAM)
 *   3. Runtime protocol (SDPCM framing, BCDC control, events)
 *   4. FullMAC control (scan, join, WPA2, disconnect)
 *
 * Reference: Linux brcmfmac driver (brcmf_sdio.c, brcmf_cfg80211.c)
 *            Circle WiFi driver (bcm4343.cpp)
 */

#pragma once
#include "types.h"

/* ── CYW43455 chip identifiers ── */
#define CYW43455_CHIP_ID        0x4345
#define CYW43455_CHIP_REV       6

/* ── Silicon Backplane (AXI) core addresses ── */
#define CYW_CHIPCOMMON_BASE     0x18000000
#define CYW_SDIO_DEV_BASE      0x18002000
#define CYW_ARM_CORE_BASE      0x18003000
#define CYW_SOCSRAM_BASE       0x18004000
#define CYW_RAM_BASE            0x00198000

/* Backplane window register (via SDIO func 1, CMD52) */
#define CYW_BAK_WIN_ADDR        0x1000A

/* ── SDPCM protocol ── */
#define SDPCM_HEADER_LEN        12
#define SDPCM_CTL_CHANNEL       0
#define SDPCM_DATA_CHANNEL      2
#define SDPCM_GLOM_CHANNEL      3
#define SDPCM_EVENT_CHANNEL     1

/* SDPCM header field offsets */
#define SDPCM_FRAMETAG_LEN      4   /* frame length + ~length */
#define SDPCM_SEQ_OFFSET        4
#define SDPCM_CHAN_OFFSET        5
#define SDPCM_NEXTLEN_OFFSET    6
#define SDPCM_DOFFSET_OFFSET    7
#define SDPCM_FLOWCTL_OFFSET    8
#define SDPCM_MAXSEQ_OFFSET     9

/* ── BCDC protocol ── */
#define BCDC_HEADER_LEN         16
#define BCDC_FLAG_SET           0x02
#define BCDC_FLAG_IF_MASK       0xF000
#define BCDC_FLAG_ID_SHIFT      16

/* BCDC iovar command IDs */
#define WLC_SET_VAR             263
#define WLC_GET_VAR             262
#define WLC_SET_SSID            26
#define WLC_GET_SSID            25
#define WLC_DISASSOC            52
#define WLC_SET_PASSIVE_SCAN    49
#define WLC_SCAN                50
#define WLC_SCAN_RESULTS        51
#define WLC_SET_WSEC            134
#define WLC_SET_WPA_AUTH        165
#define WLC_SET_INFRA           20
#define WLC_SET_AUTH            22
#define WLC_GET_RSSI            127
#define WLC_SET_PM              86
#define WLC_SET_BAND            111

/* Security modes */
#define WSEC_NONE               0
#define WSEC_WEP                1
#define WSEC_TKIP               2
#define WSEC_AES                4

#define WPA_AUTH_DISABLED       0x0000
#define WPA_AUTH_PSK            0x0004
#define WPA2_AUTH_PSK           0x0080

/* Events from firmware */
#define CYW_E_SET_SSID          0
#define CYW_E_JOIN              1
#define CYW_E_AUTH              3
#define CYW_E_DEAUTH            5
#define CYW_E_DEAUTH_IND        6
#define CYW_E_ASSOC             7
#define CYW_E_DISASSOC          11
#define CYW_E_DISASSOC_IND      12
#define CYW_E_LINK              16
#define CYW_E_PSK_SUP           46
#define CYW_E_ESCAN_RESULT      69

/* Event status codes */
#define CYW_E_STATUS_SUCCESS    0
#define CYW_E_STATUS_TIMEOUT    7
#define CYW_E_STATUS_PARTIAL    8

/* ── WiFi state ── */

#define CYW_SSID_MAX            32
#define CYW_MAC_LEN             6
#define CYW_MAX_SCAN_RESULTS    32
#define CYW_PASSPHRASE_MAX      64

/* Scan result entry */
struct cyw_scan_result {
    u8  bssid[CYW_MAC_LEN];
    u8  ssid[CYW_SSID_MAX];
    u8  ssid_len;
    u8  channel;
    i16 rssi;
    u16 capability;
    u32 security;
};

/* WiFi link state */
#define CYW_LINK_DOWN           0
#define CYW_LINK_JOINING        1
#define CYW_LINK_UP             2
#define CYW_LINK_AUTH_FAIL      3

/* ── Public API ── */

/* Phase 1: Chip init + SDIO setup */
bool cyw43_init(void);

/* Phase 2: Firmware boot */
bool cyw43_load_firmware(void);

/* Phase 3: Runtime — called from poll loop */
void cyw43_poll(void);

/* Phase 4: FullMAC control */
bool cyw43_scan_start(void);
bool cyw43_scan_get_results(struct cyw_scan_result *results, u32 *count);
bool cyw43_join(const char *ssid, u32 ssid_len,
                const char *passphrase, u32 pass_len,
                u32 security);
bool cyw43_disconnect(void);

/* Status queries */
u32  cyw43_link_state(void);
bool cyw43_is_connected(void);
i32  cyw43_get_rssi(void);
void cyw43_get_mac(u8 *mac);

/* Data path — Ethernet frames via SDPCM data channel */
bool cyw43_send_frame(const u8 *frame, u32 len);
bool cyw43_recv_frame(u8 *frame, u32 *len);
