#include "types.h"
#include "dma.h"

bool dma_cb_encode(struct dma_cb *cb, u64 src, u64 dst, u32 len,
                   bool source_increment, u64 next)
{
    const u64 address_limit = 1ULL << 40;
    const u64 cb_limit = 1ULL << 37;
    u32 source_bytes = source_increment ? len : 4U;
    if (!cb || !len || len > DMA_MAX_CB_BYTES ||
        src >= address_limit || dst >= address_limit ||
        source_bytes > address_limit - src || len > address_limit - dst ||
        (next && (next >= cb_limit || (next & 31U) != 0U)))
        return false;
    cb->ti = DMA_TI_DEST_INC | DMA_TI_WAIT_RESP |
             (source_increment ? DMA_TI_SRC_INC : 0U);
    cb->src_addr = (u32)src;
    cb->dst_addr = (u32)dst;
    cb->xfer_len = len;
    cb->stride = (u32)(src >> 32) | (u32)((dst >> 32) << 8);
    cb->next_cb = (u32)(next >> 5);
    cb->_pad[0] = cb->_pad[1] = 0;
    return true;
}
