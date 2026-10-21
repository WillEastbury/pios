#include <assert.h>
#include <stdio.h>
#include "cluster_claim.h"

int main(void)
{
    struct cluster_claim claim;
    assert(!cluster_claim_begin(&claim, 9, 8, 9));
    assert(!cluster_claim_begin(&claim, 1, 257, 1));
    assert(!cluster_claim_begin(&claim, 0, 0xffffffffU, 0));
    assert(cluster_claim_begin(&claim, 0xc0a84d01U, 0xc0a84dfdU, 0xc0a84d01U));
    u32 ip = 0;
    for (u32 i = 0; i < 253U; i++) {
        assert(cluster_claim_next_probe(&claim, &ip));
        assert(ip == 0xc0a84d01U + i);
    }
    assert(!cluster_claim_next_probe(&claim, &ip));
    cluster_claim_note_conflict(&claim, 0xc0a84d01U);
    cluster_claim_note_conflict(&claim, 0xc0a84d02U);
    assert(cluster_claim_finish(&claim) == CLUSTER_CLAIM_READY);
    assert(claim.candidate == 0xc0a84d03U);
    assert(cluster_claim_begin(&claim, 10, 12, 12));
    assert(cluster_claim_finish(&claim) == CLUSTER_CLAIM_PENDING);
    assert(cluster_claim_next_probe(&claim, &ip) && ip == 12U);
    assert(cluster_claim_next_probe(&claim, &ip) && ip == 10U);
    assert(cluster_claim_next_probe(&claim, &ip) && ip == 11U);
    assert(cluster_claim_finish(&claim) == CLUSTER_CLAIM_READY);
    cluster_claim_note_conflict(&claim, 12U);
    assert(cluster_claim_finish(&claim) == CLUSTER_CLAIM_READY);
    assert(claim.candidate == 10U);
    assert(cluster_claim_begin(&claim, 10, 11, 10));
    cluster_claim_note_conflict(&claim, 10);
    cluster_claim_note_conflict(&claim, 11);
    while (cluster_claim_next_probe(&claim, &ip)) {}
    assert(cluster_claim_finish(&claim) == CLUSTER_CLAIM_EXHAUSTED);
    puts("cluster claim: PASS");
    return 0;
}
