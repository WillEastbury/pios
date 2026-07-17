#pragma once
#include "types.h"

/*
 * pioscap.h - lightweight on-device Ethernet frame capture
 *
 * Captures RX/TX frames at the nic.c choke point (the same place
 * fb_pkt_dump() taps, so it sees every frame regardless of protocol) into a
 * fixed-size ring buffer, and can export the ring as a standard PCAP file
 * (readable directly by Wireshark/tcpdump) over HTTP. This requires no
 * host-side packet-capture driver (Npcap/WinPcap/pktmon) -- useful when the
 * dev machine is managed/locked down and cannot have one installed, or to
 * capture exactly what PIOS's own NIC driver saw rather than an external tap.
 *
 * AUTO-FREEZE: a small ring is easily evicted by unrelated ambient traffic
 * before a human gets a chance to pull it -- the original 128-entry ring was
 * gone by the time a ~45s hardware stall was diagnosed and the capture
 * downloaded. Once armed, the ring auto-freezes (stops overwriting) the
 * instant something calls pioscap_notify_event() -- wired up to the real
 * recovery paths (macb_rx_recover, macb_tx_recover_silent,
 * macb_rx_liveness_recover) -- so whatever led up to and immediately
 * followed the interesting event is preserved indefinitely until the ring
 * is explicitly cleared/re-armed, with no timing race.
 *
 * PAGINATION: sized well beyond what fits in one 16000-byte HTTP response,
 * so pioscap_export_paged() serializes a bounded window of entries at a
 * time; a client fetches pages in a loop (following pcap_status.count) and
 * concatenates the raw bytes to reassemble one valid PCAP file (only the
 * first page carries the 24-byte global header).
 */

#define PIOSCAP_MAX_ENTRIES   4096U
#define PIOSCAP_SNAPLEN       96U   /* bytes captured per frame: full Eth+IP+TCP
                                       headers (54B) plus a little payload/options */
/* Safe per-page entry count: PIOSCAP_PAGE_MAX * (16-byte pcap record header +
 * SNAPLEN) + 24-byte global header must stay well under one 16000-byte HTTP
 * response. 140 * (16 + 96) + 24 = 15704 bytes. */
#define PIOSCAP_PAGE_MAX      140U

void pioscap_init(void);
void pioscap_enable(bool on);
bool pioscap_enabled(void);
void pioscap_clear(void);

/* Called from nic.c's RX/TX choke points. Cheap no-op when disabled or
 * frozen. */
void pioscap_rx(const u8 *frame, u32 len);
void pioscap_tx(const u8 *frame, u32 len);

/* Called by any subsystem when something diagnostically interesting just
 * happened (a real NIC recovery, a stall watchdog trip, etc). If capture is
 * armed and not already frozen, this freezes the ring immediately -- no
 * further frames are recorded until pioscap_clear()/pioscap_enable() is
 * called again -- so the window of frames leading up to (and briefly after)
 * the event survives regardless of how much unrelated traffic follows.
 * `reason` must be a string literal / static string (stored as a pointer,
 * never copied or formatted, so this is safe to call from a hot path). */
void pioscap_notify_event(const char *reason);

struct pioscap_status {
    bool enabled;
    bool frozen;
    const char *freeze_reason;  /* NULL if not frozen */
    u64  freeze_ts_ms;
    u32  count;      /* frames currently held (<= PIOSCAP_MAX_ENTRIES) */
    u32  capacity;
    u64  dropped;    /* frames evicted because the ring was full (pre-freeze) */
    u64  total_rx;
    u64  total_tx;
};
void pioscap_status(struct pioscap_status *out);

/* Write the entire capture ring as a PCAP file into `out` (caller-provided
 * buffer), oldest frame first. Returns bytes written. Only safe for small
 * rings / when count is known to fit; prefer pioscap_export_paged() now that
 * the ring is sized past one HTTP response. */
u32 pioscap_export(u8 *out, u32 max);

/* Paginated export: writes entries [start_index, start_index+want) (clamped
 * to what's held) into `out`, oldest-first logical order. Only start_index
 * == 0 includes the 24-byte PCAP global header. Returns bytes written via
 * the return value; *out_written_count receives how many entries were
 * actually serialized (may be less than `want` if it wouldn't fit in `max`
 * or the ring doesn't hold that many), and *out_total_count receives the
 * total number of entries currently held (so the caller knows when to stop
 * paging). */
u32 pioscap_export_paged(u8 *out, u32 max, u32 start_index, u32 want,
                          u32 *out_written_count, u32 *out_total_count);
