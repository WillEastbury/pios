#include <assert.h>
#include <stdio.h>
#include "ipc_model.h"

static struct ipc_model_registry registry;
static struct ipc_model_request metadata(struct ipc_model_vm source,
                                         struct ipc_model_vm target, u32 shard)
{
    return (struct ipc_model_request){.model={2, 7}, .shard=shard, .tag=123,
        .source=source, .target=target, .span_id=5, .span_generation=9,
        .offset=8, .length=16, .capacity=24};
}
int main(void)
{
    struct ipc_model_registry *r = &registry;
    struct ipc_model_vm a, b, c, d;
    struct ipc_model_lane_id ab, cb, ad;
    struct ipc_model_request_id ids[8], recv[8], extra;
    struct ipc_model_request_id held[8];
    struct ipc_model_request got;
    u32 seq, count, rejected, head, tail;
    ipc_model_init(r);
    assert(ipc_model_vm_start(r, 0, &a) == IPC_OK);
    assert(ipc_model_vm_start(r, 1, &b) == IPC_OK);
    assert(ipc_model_vm_start(r, 2, &c) == IPC_OK);
    assert(ipc_model_vm_start(r, 3, &d) == IPC_OK);
    assert(ipc_model_bind(r, (struct ipc_model_id){2,7}, 10, b) == IPC_OK);
    assert(ipc_model_bind(r, (struct ipc_model_id){2,7}, 11, d) == IPC_OK);
    assert(ipc_model_bind(r, (struct ipc_model_id){3,1}, 10, d) == IPC_OK);
    assert(ipc_model_lane_open(r, a, (struct ipc_model_id){2,7}, 10, &ab) == IPC_OK);
    assert(ipc_model_lane_open(r, c, (struct ipc_model_id){2,7}, 10, &cb) == IPC_OK);
    assert(ipc_model_lane_open(r, a, (struct ipc_model_id){2,7}, 11, &ad) == IPC_OK);
    assert(ab.slot != cb.slot && ab.slot != ad.slot);
    struct ipc_model_request m = metadata(a, b, 10);
    u32 pause_sequence;
    assert(ipc_model_pause(r, b, &pause_sequence) == IPC_OK);
    assert(ipc_model_request_create(r, a, &m, &extra) == IPC_MODEL_ERR_PAUSED);
    assert(ipc_model_pause_complete(r, b, pause_sequence) == IPC_MODEL_ERR_PHASE);
    assert(ipc_model_drain_ack(r, b, pause_sequence) == IPC_OK);
    assert(ipc_model_pause_complete(r, b, pause_sequence) == IPC_OK);
    assert(ipc_model_resume(r, b, pause_sequence) == IPC_OK);
    for (u32 i = 0; i < 8; ++i) {
        m.tag = i;
        assert(ipc_model_request_create(r, a, &m, &ids[i]) == IPC_OK);
        assert(ipc_model_publish(r, c, ab, ids[i], &seq) == IPC_MODEL_ERR_OWNER);
        assert(ipc_model_publish(r, a, ad, ids[i], &seq) == IPC_ERR_INVAL);
        assert(ipc_model_publish(r, a, ab, ids[i], &seq) == IPC_OK && seq == i+1);
        assert(ipc_model_publish(r, a, ab, ids[i], &seq) == IPC_MODEL_ERR_PHASE);
        assert(ipc_model_cancel(r, a, ids[i]) == IPC_MODEL_ERR_PHASE);
    }
    assert(ipc_model_receive(r, b, ab, recv, 3, &count, &rejected) == IPC_OK);
    assert(count == 3 && rejected == 0);
    assert(ipc_model_read(r, b, recv[0], &got) == IPC_OK && got.tag == 0);
    assert(ipc_model_read(r, a, recv[0], &got) == IPC_MODEL_ERR_OWNER);
    /* Ring credit is not descriptor-pool credit: still cannot allocate. */
    for (u32 i=0; i<8; ++i)
        assert(ipc_model_request_create(r,a,&m,&held[i]) == IPC_OK);
    assert(ipc_model_request_create(r, a, &m, &extra) == IPC_ERR_NOSPC);
    assert(ipc_model_release(r, b, recv[0]) == IPC_OK);
    assert(ipc_model_read(r, b, recv[0], &got) == IPC_MODEL_ERR_STALE);
    assert(ipc_model_request_create(r, a, &m, &extra) == IPC_OK);
    assert(extra.slot == recv[0].slot && extra.generation != recv[0].generation);
    assert(ipc_model_publish(r, a, ab, extra, &seq) == IPC_OK && seq == 9);
    for (u32 i=0; i<8; ++i) assert(ipc_model_cancel(r,a,held[i]) == IPC_OK);
    m = metadata(c,b,10);
    assert(ipc_model_request_create(r,c,&m,&extra) == IPC_OK);
    assert(ipc_model_publish(r,c,cb,extra,&seq) == IPC_OK && seq == 1);
    assert(ipc_model_receive(r,b,cb,recv,8,&count,&rejected) == IPC_OK && count == 1);
    assert(ipc_model_cancel(r,b,recv[0]) == IPC_OK);
    assert(ipc_model_receive(r,b,cb,recv,8,&count,&rejected) == IPC_ERR_EMPTY && count == 0);
    assert(ipc_model_sequences(r,a,ab,&head,&tail) == IPC_OK && head == 9 && tail == 3);
    assert(ipc_model_receive(r,d,ab,recv,8,&count,&rejected) == IPC_MODEL_ERR_OWNER);
    assert(ipc_model_receive(r,b,ab,recv,8,&count,&rejected) == IPC_OK && count == 6);
    for (u32 i=0; i<count; ++i) assert(ipc_model_release(r,b,recv[i]) == IPC_OK);
    assert(ipc_model_release(r,b,ids[1]) == IPC_OK);
    assert(ipc_model_release(r,b,ids[2]) == IPC_OK);
    m = metadata(c,b,10);
    assert(ipc_model_request_create(r,c,&m,&extra) == IPC_OK);
    assert(ipc_model_publish(r,c,cb,extra,&seq) == IPC_OK);
    assert(ipc_model_pause(r,b,&pause_sequence) == IPC_OK);
    assert(ipc_model_drain_ack(r,b,pause_sequence) == IPC_MODEL_ERR_PHASE);
    assert(ipc_model_request_create(r,c,&m,&held[0]) == IPC_MODEL_ERR_PAUSED);
    assert(ipc_model_receive(r,b,cb,recv,1,&count,&rejected) == IPC_OK &&
           count == 1);
    assert(ipc_model_release(r,b,recv[0]) == IPC_OK);
    assert(ipc_model_drain_ack(r,b,pause_sequence) == IPC_OK);
    assert(ipc_model_pause_complete(r,b,pause_sequence) == IPC_OK);
    assert(ipc_model_resume(r,b,pause_sequence) == IPC_OK);
    /* Malformed spans fail before any descriptor allocation. */
    m = metadata(a,b,10); m.offset = ~0U;
    assert(ipc_model_request_create(r,a,&m,&extra) == IPC_ERR_TOOLONG);
    m = metadata(a,b,10); m.length = 17;
    assert(ipc_model_request_create(r,a,&m,&extra) == IPC_ERR_TOOLONG);
    m = metadata(a,b,10);
    assert(ipc_model_request_create(r,a,&m,&extra) == IPC_OK);
    assert(ipc_model_cancel(r,a,extra) == IPC_OK);
    assert(ipc_model_publish(r,a,ab,extra,&seq) == IPC_MODEL_ERR_STALE);
    /* Full queue leaves descriptor owned, and sequence/output unchanged. */
    for (u32 i=0; i<8; ++i) {
        assert(ipc_model_request_create(r,a,&m,&ids[i]) == IPC_OK);
        assert(ipc_model_publish(r,a,ab,ids[i],&seq) == IPC_OK);
    }
    assert(ipc_model_request_create(r,a,&m,&extra) == IPC_OK);
    seq = 99;
    assert(ipc_model_publish(r,a,ab,extra,&seq) == IPC_ERR_FULL && seq == 99);
    assert(ipc_model_read(r,a,extra,&got) == IPC_OK);
    assert(ipc_model_vm_retire(r,b) == IPC_OK);
    struct ipc_model_vm newer;
    assert(ipc_model_vm_start(r,1,&newer) == IPC_OK && newer.generation != b.generation);
    assert(ipc_model_bind(r,(struct ipc_model_id){2,6},10,newer) == IPC_MODEL_ERR_STALE);
    assert(ipc_model_bind(r,(struct ipc_model_id){2,7},10,newer) == IPC_OK);
    /* Generation 7 remains live via shard 11; fresh VM gets fresh lane. */
    struct ipc_model_lane_id fresh;
    assert(ipc_model_lane_open(r,a,(struct ipc_model_id){2,7},10,&fresh) == IPC_OK);
    assert(ipc_model_publish(r,a,ab,extra,&seq) == IPC_MODEL_ERR_STALE);
    assert(fresh.generation != ab.generation);
    m = metadata(a,newer,10);
    assert(ipc_model_request_create(r,a,&m,&extra) == IPC_OK);
    assert(ipc_model_publish(r,a,fresh,extra,&seq) == IPC_OK);
    /* Fault injection: consumer must reject invalid authority bounds, not
     * materialize a request or try to repair the descriptor. */
    r->requests[extra.slot].request.length = ~0U;
    assert(ipc_model_receive(r,newer,fresh,recv,1,&count,&rejected) == IPC_OK);
    assert(count == 0 && rejected == 1);
    assert(ipc_model_cancel(r,a,extra) == IPC_MODEL_ERR_PHASE);
    assert(ipc_model_sequences(r,a,fresh,&head,&tail) == IPC_OK && head == tail);
    assert(ipc_model_vm_retire(r,a) == IPC_OK);
    assert(ipc_model_read(r,newer,extra,&got) == IPC_MODEL_ERR_STALE);
    puts("ipc_model: PASS");
    return 0;
}
