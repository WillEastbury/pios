#pragma once
#include "types.h"

#define CLUSTER_CLAIM_MAX_HOSTS 256U

enum cluster_claim_result {
    CLUSTER_CLAIM_PENDING = 0,
    CLUSTER_CLAIM_READY,
    CLUSTER_CLAIM_EXHAUSTED,
    CLUSTER_CLAIM_INVALID
};

struct cluster_claim {
    u32 range_start;
    u32 range_end;
    u32 preferred;
    u32 cursor;
    u32 candidate;
    u32 hosts;
    u8 occupied[CLUSTER_CLAIM_MAX_HOSTS / 8U];
    bool probing_complete;
    enum cluster_claim_result result;
} ALIGNED(64);

bool cluster_claim_begin(struct cluster_claim *claim, u32 range_start,
                         u32 range_end, u32 preferred);
bool cluster_claim_next_probe(struct cluster_claim *claim, u32 *ip_out);
void cluster_claim_note_conflict(struct cluster_claim *claim, u32 ip);
enum cluster_claim_result cluster_claim_finish(struct cluster_claim *claim);
