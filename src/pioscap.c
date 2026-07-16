/*
 * pioscap.c - lightweight on-device Ethernet frame capture ring buffer.
 *
 * See pioscap.h. Captures a snapshot of every RX/TX frame at the nic.c
 * choke point into a small fixed-size ring, and can export it as a
 * standards-compliant PCAP file for offline analysis in Wireshark -- with
 * no host-side capture driver required.
 *
 * Owned exclusively by Core 0 (the only core that ever touches nic.c), so
 * no cross-core synchronization is needed for the ring buffer.
 */

#include "pioscap.h"
#include "timer.h"
#include "simd.h"

struct pioscap_entry {
    u64 ts_ms;
    u16 len;      /* on-wire frame length */
    u16 caplen;   /* captured length, <= PIOSCAP_SNAPLEN */
    u8  dir;      /* 0 = RX, 1 = TX */
    u8  _pad[3];
    u8  data[PIOSCAP_SNAPLEN];
};

static struct pioscap_entry ring[PIOSCAP_MAX_ENTRIES];
static u32 ring_head;    /* next write index */
static u32 ring_count;   /* valid entries, <= PIOSCAP_MAX_ENTRIES */
static bool cap_enabled;
static u64 dropped_count;
static u64 total_rx_count;
static u64 total_tx_count;

void pioscap_init(void)
{
    cap_enabled = false;
    ring_head = 0;
    ring_count = 0;
    dropped_count = 0;
    total_rx_count = 0;
    total_tx_count = 0;
}

void pioscap_enable(bool on) { cap_enabled = on; }
bool pioscap_enabled(void)   { return cap_enabled; }

void pioscap_clear(void)
{
    ring_head = 0;
    ring_count = 0;
    dropped_count = 0;
}

static void pioscap_capture(u8 dir, const u8 *frame, u32 len)
{
    if (!cap_enabled || !frame || len == 0)
        return;
    struct pioscap_entry *e = &ring[ring_head];
    e->ts_ms = timer_monotonic_ms();
    e->len = (u16)(len > 0xFFFFU ? 0xFFFFU : len);
    u32 caplen = len < PIOSCAP_SNAPLEN ? len : PIOSCAP_SNAPLEN;
    e->caplen = (u16)caplen;
    e->dir = dir;
    simd_memcpy(e->data, frame, caplen);
    ring_head = (ring_head + 1U) % PIOSCAP_MAX_ENTRIES;
    if (ring_count < PIOSCAP_MAX_ENTRIES)
        ring_count++;
    else
        dropped_count++;
}

void pioscap_rx(const u8 *frame, u32 len)
{
    total_rx_count++;
    pioscap_capture(0, frame, len);
}

void pioscap_tx(const u8 *frame, u32 len)
{
    total_tx_count++;
    pioscap_capture(1, frame, len);
}

void pioscap_status(struct pioscap_status *out)
{
    if (!out)
        return;
    out->enabled = cap_enabled;
    out->count = ring_count;
    out->capacity = PIOSCAP_MAX_ENTRIES;
    out->dropped = dropped_count;
    out->total_rx = total_rx_count;
    out->total_tx = total_tx_count;
}

/* ---- PCAP export (standard libpcap file format, LINKTYPE_ETHERNET) ---- */

struct PACKED pcap_global_hdr {
    u32 magic;
    u16 ver_major;
    u16 ver_minor;
    i32 thiszone;
    u32 sigfigs;
    u32 snaplen;
    u32 network;
};

struct PACKED pcap_rec_hdr {
    u32 ts_sec;
    u32 ts_usec;
    u32 incl_len;
    u32 orig_len;
};

u32 pioscap_export(u8 *out, u32 max)
{
    if (!out || max < sizeof(struct pcap_global_hdr))
        return 0;

    u32 off = 0;
    struct pcap_global_hdr gh;
    gh.magic = 0xA1B2C3D4U;
    gh.ver_major = 2;
    gh.ver_minor = 4;
    gh.thiszone = 0;
    gh.sigfigs = 0;
    gh.snaplen = PIOSCAP_SNAPLEN;
    gh.network = 1; /* LINKTYPE_ETHERNET */
    simd_memcpy(out + off, &gh, sizeof(gh));
    off += (u32)sizeof(gh);

    u32 n = ring_count;
    u32 start = (ring_head + PIOSCAP_MAX_ENTRIES - n) % PIOSCAP_MAX_ENTRIES;
    for (u32 i = 0; i < n; i++) {
        struct pioscap_entry *e = &ring[(start + i) % PIOSCAP_MAX_ENTRIES];
        u32 need = (u32)sizeof(struct pcap_rec_hdr) + e->caplen;
        if (off + need > max)
            break; /* ring is sized so this should not happen in practice */
        struct pcap_rec_hdr rh;
        u64 ts_ms = e->ts_ms;
        rh.ts_sec = (u32)(ts_ms / 1000ULL);
        rh.ts_usec = (u32)((ts_ms % 1000ULL) * 1000ULL);
        rh.incl_len = e->caplen;
        rh.orig_len = e->len;
        simd_memcpy(out + off, &rh, sizeof(rh));
        off += (u32)sizeof(rh);
        simd_memcpy(out + off, e->data, e->caplen);
        off += e->caplen;
    }
    return off;
}
