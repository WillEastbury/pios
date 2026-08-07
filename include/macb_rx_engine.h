/*
 * macb_rx_engine.h - Cadence GEM (RP1, BCM2712 / Raspberry Pi 5) RX descriptor
 * engine.
 *
 * Standalone RX-ring state machine extracted from the MACB driver. The driver
 * (src/macb.c) owns the fixed DMA_NET ring/buffer layout and the PHY/NCFGR
 * bring-up; it hands both to the engine via macb_rx_engine_init() and then
 * drives receive plus layered recovery through the calls below.
 *
 * Ownership contract: once initialised the engine is the exclusive owner of the
 * RX ring OWN bits, the software RX cursor, RBQP/RBQPH programming, NCR.RE for
 * RX restart, and the RSR BNA/OVR/HRESP fault bits (clearing + persistence).
 * The driver must not clear those RSR bits or rewrite the RX ring behind it.
 */
#pragma once
#include "types.h"

/* Ring/buffer geometry and offload state handed over by the driver. */
struct macb_rx_config {
    volatile void *ring;    /* 64-byte-aligned descriptor ring base            */
    u8 *buffers;            /* contiguous RX buffer pool (ring_count buffers)   */
    u32 ring_count;         /* descriptors == buffers                           */
    u32 buffer_size;        /* bytes per RX buffer                              */
    u64 ring_dma;           /* full device address of descriptor ring           */
    u64 buffers_dma;        /* full device address of contiguous buffer pool    */
    u32 trailing_bytes;     /* bytes after the logical ring before next object  */
    bool checksum_enabled;  /* RX checksum offload currently active in NCFGR    */
};

/* Read-only health snapshot (never mutates ring or hardware). */
struct macb_rx_diag {
    u32 rx_idx;                 /* software RX cursor                           */
    u32 rx_owned;               /* descriptors currently OWN=software           */
    u32 rx_contig_owned;        /* contiguous OWN run starting at rx_idx        */
    u32 rx_owned_after_gap;     /* OWN descriptors after the first non-OWN gap  */
    u32 rx_first_owned_distance;/* distance to first OWN after a gap (== depth
                                 * when there is none)                          */
    u32 rx_recv;                /* lifetime frames received                     */
    u32 rx_recover;             /* lifetime BNA/OVR/HRESP status recoveries     */
    u32 rx_hole_recover;        /* lifetime ordered-ring hole recoveries        */
    u32 rx_live_recover;        /* lifetime liveness (silence) recoveries       */
    u32 rx_wedge;               /* lifetime real wedges: silence + unmet demand */
    u32 rx_idle;                /* lifetime extended RX-silence periods (info)  */
    u32 rsr;
    u32 ncr;
    u32 rbqp;
    u32 rbqph;
};

/* Maximum later-OWN descriptors captured in an ordered-hole image. */
#define MACB_RX_CAPTURE_OWNED 8U

struct macb_rx_desc_image {
    u32 index;
    u32 addr;
    u32 ctrl;
    u32 addr_hi;
    u32 word3;
};

/* Raw descriptor image captured the moment a stable ordered-ring hole is
 * confirmed, preserved across the subsequent ring rebuild for later inspection. */
struct macb_rx_hole_image {
    bool valid;
    u32 sequence;
    u32 stuck_idx;
    u32 expected_addr;
    u32 rbqp;
    u32 rbqph;
    u32 rsr;
    u32 ncr;
    u32 rbqp_stopped;
    u32 dmacfg;
    u32 dcfg10;
    u32 rxbdctrl;
    u32 prefetch_descs;
    u32 trailing_bytes;
    u32 cache_line;
    u32 cache_probe_flags;
    struct macb_rx_desc_image stuck;
    struct macb_rx_desc_image stopped;
    struct macb_rx_desc_image after_ivac;
    u32 later_owned_count;
    struct macb_rx_desc_image later_owned[MACB_RX_CAPTURE_OWNED];
};

#define MACB_RX_CACHE_PROBE_STOPPED          (1U << 0)
#define MACB_RX_CACHE_PROBE_WORDS_CHANGED    (1U << 1)
#define MACB_RX_CACHE_PROBE_OWN_BEFORE_IVAC  (1U << 2)
#define MACB_RX_CACHE_PROBE_OWN_AFTER_IVAC   (1U << 3)

/* Publish the ring and enable reception. Returns false (fail closed) if the
 * config is invalid, DMACFG is not in 4-word ADDR64 mode, or NCR.RE cannot be
 * proven disabled before the ring is written. */
bool macb_rx_engine_init(const struct macb_rx_config *config);

/* Drain one completed descriptor into frame[0..frame_capacity). Sets
 * *frame_length and *checksum_trusted. Returns true only when a well-formed
 * frame was copied; malformed completions are released and reported false. */
bool macb_rx_engine_recv(u8 *frame, u32 frame_capacity, u32 *frame_length,
                         bool *checksum_trusted);

/* Layered recovery. Each is cheap to call every poll and returns true only when
 * it performed a destructive ring rebuild. */
bool macb_rx_engine_recover_status(void);        /* BNA/OVR/HRESP fault owner   */
bool macb_rx_engine_recover_ordering_hole(void); /* impossible ordered hole     */
bool macb_rx_engine_recover_liveness(u64 now_ms, u32 tx_progress_counter);

/* Read-only diagnostics + last captured hole image. */
void macb_rx_engine_diag(struct macb_rx_diag *out);
bool macb_rx_engine_last_hole(struct macb_rx_hole_image *out);

/* Track a runtime change to RX checksum offload so recv() reports trust
 * consistently with NCFGR.RXCOEN. */
void macb_rx_engine_set_checksum_enabled(bool enable);
