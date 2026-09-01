from pathlib import Path


root = Path(__file__).resolve().parent.parent
net = (root / "src" / "net.c").read_text(encoding="utf-8")
start = net.index("void net_handle_fifo_request(void)")
end = net.index("\n}\n", start) + 3
body = net[start:end]

assert "requester = CORE_USERM" in body
assert "requester <= CORE_USER1" in body
assert "fifo_pop_batch(CORE_NET, requester" in body
assert "if (msg.type == MSG_NET_UDP_SEND)" in body
assert "reply.type = MSG_NET_UDP_DONE" in body
assert "reply.status = ok ? 0U : 1U" in body
assert "fifo_push(CORE_NET, requester, &reply)" in body
assert "else if (msg.type == MSG_DNS_RESOLVE)" in body
assert "dns_fifo_reply(requester, msg.tag, 1U, 0U)" in body
assert "proc_buffer_ref_acquire(requester, msg.buffer" in body
assert "dcache_invalidate_range(msg.buffer, msg.length);" in body
assert "socket_handle_message(requester, &msg)" in body
assert "walfs_handle_message(requester, &msg)" in body
assert "socket_handle_fifo(CORE_USER0)" not in body
assert "socket_handle_fifo(CORE_USER1)" not in body

print("net fifo: cores 1-3 drained, malformed requests reply, and service messages are demultiplexed")
