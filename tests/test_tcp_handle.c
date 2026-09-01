#include <assert.h>
#include "tcp.h"

int main(void)
{
    tcp_conn_t a = tcp_conn_make(7U, 1U);
    tcp_conn_t b = tcp_conn_make(7U, 2U);
    u32 slot = 0U;
    u32 generation = 0U;

    assert(a >= 0);
    assert(b >= 0);
    assert(a != b);
    assert(tcp_conn_decode(a, &slot, &generation));
    assert(slot == 7U);
    assert(generation == 1U);
    assert(tcp_conn_decode(b, &slot, &generation));
    assert(slot == 7U);
    assert(generation == 2U);
    assert(!tcp_conn_decode(TCP_CONN_INVALID, &slot, &generation));
    assert(tcp_conn_make(TCP_CONN_INDEX_MASK + 1U, 1U) ==
           TCP_CONN_INVALID);
    assert(tcp_conn_make(0U, 0U) == TCP_CONN_INVALID);
    return 0;
}
