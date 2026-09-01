from pathlib import Path


root = Path(__file__).resolve().parent.parent
net_h = (root / "include" / "net.h").read_text(encoding="utf-8")
fifo_h = (root / "include" / "fifo.h").read_text(encoding="utf-8")
socket_h = (root / "include" / "socket.h").read_text(encoding="utf-8")
net = (root / "src" / "net.c").read_text(encoding="utf-8")
socket = (root / "src" / "socket.c").read_text(encoding="utf-8")
dns = (root / "src" / "dns.c").read_text(encoding="utf-8")
dhcp = (root / "src" / "dhcp.c").read_text(encoding="utf-8")
kernel = (root / "src" / "kernel.c").read_text(encoding="utf-8")

assert "udp_recv_cb)(nic_iface_t iface, u32 dst_ip" in net_h
assert "_reserved=destination IPv4 address" in fifo_h
assert "nic_iface_t iface;" in socket_h
assert "udp_callback(ingress_iface, dst_ip" in net
assert "cb(ingress_iface, dst_ip" in net
assert "return net_send_udp_on(net_current_iface" in net
assert "u32 dst_ip = msg->_reserved" in socket
assert "s->local.ip == 0U || s->local.ip == dst_ip" in socket
assert "s->iface == NIC_IFACE_ANY || s->iface == iface" in socket
assert "src->iface = (nic_iface_t)msg.iface" in socket
assert "nic_iface_t iface, u32 dst_ip" in dns
assert "nic_iface_t iface, u32 dst_ip" in dhcp
assert "net_send_udp_on(iface, src_ip" in kernel

print("udp iface: callback, socket and FIFO metadata carry ingress identity")
