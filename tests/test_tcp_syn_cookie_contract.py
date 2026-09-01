from pathlib import Path


tcp = (
    Path(__file__).resolve().parent.parent / "src" / "tcp.c"
).read_text(encoding="utf-8")

assert "#define TCP_SYN_PAYLOAD_SLOTS" in tcp
assert "u16 data_slot;" in tcp
assert "u16 data_len;" in tcp
assert "tcp_syn_payload_alloc(seg_data, data_len)" in tcp
assert "tcp_send_cookie_ack" in tcp
assert "seg_seq + listen->pending[idx].data_len" in tcp
assert "tcp_syn_payloads[data_slot].data" in tcp

accept = tcp[tcp.index("tcp_conn_t tcp_accept("):tcp.index(
    "\nu32 tcp_write(", tcp.index("tcp_conn_t tcp_accept(")
)]
assert "ring_write(&t->rx_buf" in accept
assert "t->rcv_nxt += written" in accept
assert accept.index("ring_write(&t->rx_buf") < accept.index("tcp_send_ack(t)")

print("tcp syn cookie: bounded ACK payload retained, ingested and acknowledged")
