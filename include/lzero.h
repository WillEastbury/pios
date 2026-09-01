/*
 * lzero.h - Intel LevelZero tensor backend (fail-closed)
 *
 * Path: B compute-class seen → C BAR sizes / optional BAR0 map →
 *       D GuC firmware → E known-answer proof → VERIFIED (green).
 *
 * QPU word arrays are not L0 modules. No 16 GiB LMEM map, no MSI, no
 * i915/xe, no tick poll. VERIFIED is never produced by classify().
 */
#pragma once
#include "types.h"
#include "pcie1.h"

#define LZERO_DASH_BACKEND  "PCIE GPU"
#define LZERO_DASH_CAPS \
    "Tensor PCIe GPU External -> Intel LevelZero Acceleration"

/* Gates on the B→E path. 0 = not at B. */
#define LZERO_GATE_A    0U
#define LZERO_GATE_B    1U
#define LZERO_GATE_C    2U
#define LZERO_GATE_D    3U
#define LZERO_GATE_E    4U
#define LZERO_GATE_DONE 5U

#define LZERO_BLOCK_NONE        0U
#define LZERO_BLOCK_NO_GPU      1U
#define LZERO_BLOCK_NO_BARS     2U
#define LZERO_BLOCK_ATU         3U
#define LZERO_BLOCK_NO_MAP      4U
#define LZERO_BLOCK_NO_GUC_FW   5U
#define LZERO_BLOCK_NO_PROOF    6U
#define LZERO_BLOCK_PROOF_FAIL  7U

enum lzero_state {
    LZERO_OFF = 0,
    LZERO_NO_PCIE1,
    LZERO_NO_LINK,
    LZERO_NO_DEVICE,
    LZERO_NOT_B50,
    LZERO_GPU_SEEN,
    LZERO_B50_SEEN,
    LZERO_VERIFIED
};

struct lzero_status {
    u32 state;
    u32 gate;
    u32 block;
    bool pcie1_up;
    bool b50_found;
    bool gpu_found;
    bool bars_probed;
    bool bar0_fits_atu;
    bool bar0_mapped;
    bool guc_fw_present;
    bool guc_ready;
    bool proof_ok;
    u8 gpu_bus;
    u8 gpu_dev;
    u8 gpu_func;
    u8 gpu_class;
    u8 gpu_sub;
    u16 vendor_id;
    u16 device_id;
    u32 link_speed;
    u32 link_width;
    u64 bar0_size;
    u64 lmem_size;
    u64 atu_size;
    const char *next;
};

struct lzero_facts {
    bool gpu_seen;
    bool bars_probed;
    bool bar0_mapped;
    bool bar0_fits_atu;
    bool guc_fw_present;
    bool guc_ready;
    bool proof_ok;
};

static inline u32 lzero_classify_ex(bool has_pcie1, bool link_up, bool any_ep,
                                    bool compute, bool b50)
{
    if (!has_pcie1)
        return LZERO_NO_PCIE1;
    if (!link_up)
        return LZERO_NO_LINK;
    if (!any_ep)
        return LZERO_NO_DEVICE;
    if (b50)
        return LZERO_B50_SEEN;
    if (compute)
        return LZERO_GPU_SEEN;
    return LZERO_NOT_B50;
}

/* DID-only helper kept for host tests. */
static inline u32 lzero_classify(bool has_pcie1, bool link_up, bool any_ep,
                                 u16 vendor, u16 device)
{
    bool b50 = pcie1_is_b50(vendor, device);
    return lzero_classify_ex(has_pcie1, link_up, any_ep, b50, b50);
}

static inline u32 lzero_gate_from_facts(const struct lzero_facts *f)
{
    if (!f || !f->gpu_seen)
        return LZERO_GATE_A;
    if (!f->bars_probed)
        return LZERO_GATE_B;
    if (!f->bar0_mapped)
        return LZERO_GATE_C;
    if (!f->guc_ready)
        return LZERO_GATE_D;
    if (!f->proof_ok)
        return LZERO_GATE_E;
    return LZERO_GATE_DONE;
}

static inline u32 lzero_block_from_facts(const struct lzero_facts *f)
{
    if (!f || !f->gpu_seen)
        return LZERO_BLOCK_NO_GPU;
    if (!f->bars_probed)
        return LZERO_BLOCK_NO_BARS;
    if (!f->bar0_fits_atu)
        return LZERO_BLOCK_ATU;
    if (!f->bar0_mapped)
        return LZERO_BLOCK_NO_MAP;
    if (!f->guc_fw_present || !f->guc_ready)
        return LZERO_BLOCK_NO_GUC_FW;
    if (!f->proof_ok)
        return LZERO_BLOCK_NO_PROOF;
    return LZERO_BLOCK_NONE;
}

static inline const char *lzero_gate_name(u32 gate)
{
    switch (gate) {
    case LZERO_GATE_B:    return "B";
    case LZERO_GATE_C:    return "C";
    case LZERO_GATE_D:    return "D";
    case LZERO_GATE_E:    return "E";
    case LZERO_GATE_DONE: return "done";
    default:              return "A";
    }
}

static inline const char *lzero_block_name(u32 block)
{
    switch (block) {
    case LZERO_BLOCK_NONE:       return "none";
    case LZERO_BLOCK_NO_GPU:     return "no-compute-fn";
    case LZERO_BLOCK_NO_BARS:    return "probe-bars";
    case LZERO_BLOCK_ATU:        return "bar0>atu (not LMEM)";
    case LZERO_BLOCK_NO_MAP:     return "lzero map (BAR0 only)";
    case LZERO_BLOCK_NO_GUC_FW:  return "guc firmware missing";
    case LZERO_BLOCK_NO_PROOF:   return "no known-answer ZEBIN";
    case LZERO_BLOCK_PROOF_FAIL: return "proof mismatch";
    default:                     return "?";
    }
}

static inline const char *lzero_next_from_facts(const struct lzero_facts *f)
{
    switch (lzero_block_from_facts(f)) {
    case LZERO_BLOCK_NO_GPU:    return "pcie1 scan: wait for class 03:02/12:00";
    case LZERO_BLOCK_NO_BARS:   return "lzero probe (config BAR sizes)";
    case LZERO_BLOCK_ATU:       return "grow pcie1 ATU to fit BAR0; never map LMEM";
    case LZERO_BLOCK_NO_MAP:    return "lzero map (BAR0 into 32MiB Device ATU)";
    case LZERO_BLOCK_NO_GUC_FW: return "WALFS guc blob + adrv load (not yet)";
    case LZERO_BLOCK_NO_PROOF:  return "host-compiled ZEBIN known-answer";
    default:                    return "verified";
    }
}

static inline u32 lzero_dash_color(u32 state)
{
    return state == LZERO_VERIFIED ? 0x0000FF80U : 0x00FF4040U;
}

static inline const char *lzero_dash_state(u32 state)
{
    if (state == LZERO_VERIFIED)
        return "verified";
    if (state == LZERO_B50_SEEN || state == LZERO_GPU_SEEN)
        return "seen";
    return "off";
}

static inline const char *lzero_dash_state_of(const struct lzero_status *s)
{
    if (!s)
        return "off";
    if (s->state == LZERO_VERIFIED || s->proof_ok)
        return "verified";
    if (s->gate == LZERO_GATE_E)
        return "E";
    if (s->gate == LZERO_GATE_D)
        return "D";
    if (s->gate == LZERO_GATE_C)
        return "C";
    if (s->gate == LZERO_GATE_B || s->gpu_found)
        return "B";
    return lzero_dash_state(s->state);
}

static inline const char *lzero_state_name(u32 state)
{
    switch (state) {
    case LZERO_NO_PCIE1:  return "no-pcie1";
    case LZERO_NO_LINK:   return "no-link";
    case LZERO_NO_DEVICE: return "no-device";
    case LZERO_NOT_B50:   return "not-gpu";
    case LZERO_GPU_SEEN:  return "gpu-seen";
    case LZERO_B50_SEEN:  return "b50-seen";
    case LZERO_VERIFIED:  return "verified";
    default:              return "off";
    }
}

static inline const struct pcie1_ep *lzero_pick_compute(const struct pcie1_ep *eps,
                                                       u32 n)
{
    u32 i;
    if (!eps)
        return NULL;
    for (i = 0; i < n; i++) {
        if (pcie1_is_compute_ep(&eps[i]))
            return &eps[i];
    }
    return NULL;
}

void lzero_probe(void);
void lzero_status(struct lzero_status *out);
bool lzero_probe_bars(void);
bool lzero_map_bar0(void);
