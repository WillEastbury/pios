from pathlib import Path


tcp = (
    Path(__file__).resolve().parent.parent / "src" / "tcp.c"
).read_text(encoding="utf-8")

assert "u32 generation;" in tcp
assert "static struct tcb *tcb_from_handle" in tcp
assert "t->generation != generation" in tcp
assert "tcb_bump_generation(t->generation)" in tcp
assert "table[i].generation = 1U" in tcp
assert "out[n].conn = tcb_handle(t)" in tcp
assert "return tcb_handle(t)" in tcp
assert "tcbs[conn]" not in tcp
assert "tcbs[listen_conn]" not in tcp

for api in (
    "tcp_iface", "tcp_accept", "tcp_write", "tcp_advertise_window",
    "tcp_read", "tcp_close", "tcp_abort", "tcp_state", "tcp_readable",
    "tcp_writable", "tcp_tx_pending", "tcp_peek",
):
    start = tcp.index(f"{api}(")
    body = tcp[start:tcp.index("\n}", start) + 2]
    assert "tcb_from_handle" in body, api

print("tcp handles: every public lookup validates slot generation")
