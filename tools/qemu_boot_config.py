#!/usr/bin/env python3
"""Verify FAT-root provisioning against the real PIOS kernel under QEMU."""
from __future__ import annotations

import argparse
import pathlib
import socket
import struct
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.request

from build_qemu_disk_image import build_image
from debug_console_qemu_test import DebugConn

ROOT = pathlib.Path(__file__).resolve().parent.parent
NETWORK = (
    "[network]\nip=192.168.0.210\nmask=255.255.255.0\n"
    "gateway=192.168.0.1\ndns=192.168.0.53\n"
)
# Public fixture key, never used to enroll a real node.
FIXTURE_KEY = "0123456789abcdef" * 4
CLUSTER = (
    f"[cluster]\nname=Test-Lab\ncluster_id=00112233445566778899aabbccddeeff\n"
    f"root_bootstrap_key={FIXTURE_KEY}\n"
    "range_start=192.168.77.1\nrange_end=192.168.77.253\n"
    "seed1=192.168.77.1\nseed2=192.168.77.2\nseed3=192.168.77.3\n"
)
CLUSTER_NETWORK = (
    "[network]\nip=192.168.77.4\nmask=255.255.255.0\n"
    "gateway=192.168.77.254\ndns=192.168.77.254\n"
)


class ClaimPeer:
    """Real Ethernet peer on QEMU's hub, not a mocked kernel claim result."""

    def __init__(self, sock: socket.socket, occupied: str | None,
                 simultaneous: bool = False, late: bool = False) -> None:
        self.sock = sock
        self.occupied = socket.inet_aton(occupied) if occupied else None
        self.simultaneous = simultaneous
        self.late = late
        self.probes: dict[bytes, int] = {}
        self.frames: list[bytes] = []
        self.error: Exception | None = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self) -> None:
        pending = bytearray()
        self.sock.settimeout(0.2)
        try:
            while not self.stopping.is_set():
                try:
                    data = self.sock.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                pending.extend(data)
                while len(pending) >= 4:
                    length = struct.unpack_from("!I", pending)[0]
                    if length > 1518:
                        raise AssertionError(f"oversized peer frame: {length}")
                    if len(pending) < length + 4:
                        break
                    frame = bytes(pending[4:length + 4])
                    del pending[:length + 4]
                    self.frames.append(frame)
                    if (len(frame) < 42 or frame[12:14] != b"\x08\x06" or
                            frame[20:22] != b"\0\1" or frame[28:32] != b"\0" * 4):
                        continue
                    target = frame[38:42]
                    self.probes[target] = self.probes.get(target, 0) + 1
                    if target != self.occupied or (self.late and self.probes[target] <= 3):
                        continue
                    mac = b"\x02\x11\x22\x33\x44\x55"
                    reply = (b"\xff" * 6 + mac + b"\x08\x06" +
                             b"\0\1\x08\0\x06\x04\0\1" + mac +
                             (b"\0" * 4 if self.simultaneous else target) +
                             b"\0" * 6 + target)
                    reply = reply.ljust(60, b"\0")
                    self.sock.sendall(struct.pack("!I", len(reply)) + reply)
        except Exception as error:
            if not self.stopping.is_set():
                self.error = error

    def close(self) -> None:
        self.stopping.set()
        self.thread.join(timeout=2)
        self.sock.close()
        if self.thread.is_alive():
            raise RuntimeError("claim peer failed to stop")
        if self.error:
            raise self.error


def check_boot(qemu: pathlib.Path, kernel: pathlib.Path, folder: pathlib.Path,
               name: str, config: bytes | None, mode: str,
               marker: str, occupied: str | None = None,
               simultaneous: bool = False, late: bool = False) -> None:
    disk = folder / f"{name}.img"
    serial = folder / f"{name}.log"
    stderr = folder / f"{name}.stderr"
    build_image(kernel.read_bytes(), disk, 0x43464731, boot_config=config)
    if mode == "static":
        address = "192.168.0.210"
        expected = ("ip=192.168.0.210", "mask=255.255.255.0",
                    "gw=192.168.0.1", "dns=192.168.0.53")
    elif mode == "recovery":
        address = "192.168.0.201"
        expected = ("ip=192.168.0.201", "mask=255.255.0.0",
                    "gw=192.168.0.1", "dns=192.168.0.1")
    elif mode in ("dhcp", "decline"):
        address = "192.168.0.15"
        expected = ("ip=192.168.0.15", "mask=255.255.255.0",
                    "gw=192.168.0.1", "dns=192.168.0.3")
    elif mode == "cluster":
        address = "192.168.77.5" if occupied else "192.168.77.4"
        expected = (f"ip={address}", "mask=255.255.255.0",
                    "gw=192.168.77.254", "dns=192.168.77.254")
    else:
        raise ValueError(f"unsupported boot mode: {mode}")
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        port = reserve.getsockname()[1]
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        console_port = reserve.getsockname()[1]
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        peer_port = reserve.getsockname()[1]
    capsule_ports = {}
    for guest_port in (82, 83, 8090):
        with socket.socket() as reserve:
            reserve.bind(("127.0.0.1", 0))
            capsule_ports[guest_port] = reserve.getsockname()[1]
    subnet, host = ("192.168.77.0/24", "192.168.77.254") if mode == "cluster" else (
        "192.168.0.0/24", "192.168.0.1")
    network = (
        f"user,id=n0,net={subnet},host={host},"
        f"hostfwd=tcp:127.0.0.1:{port}-{address}:80,"
        f"hostfwd=tcp:127.0.0.1:{console_port}-{address}:2323"
    )
    network += "".join(
        f",hostfwd=tcp:127.0.0.1:{host_port}-{address}:{guest_port}"
        for guest_port, host_port in capsule_ports.items()
    )
    command = [
        str(qemu), "-M", "virt", "-cpu", "cortex-a76", "-smp", "4", "-m", "1G",
        "-display", "none", "-serial", f"file:{serial}", "-S", "-monitor", "stdio",
        "-kernel", str(kernel), "-netdev", network,
        "-netdev", "hubport,id=user-port,hubid=0,netdev=n0",
        "-netdev", "hubport,id=guest-port,hubid=0",
        "-device", "virtio-net-device,netdev=guest-port",
        "-netdev", f"socket,id=peer,listen=127.0.0.1:{peer_port}",
        "-netdev", "hubport,id=peer-port,hubid=0,netdev=peer",
        "-drive", f"if=none,format=raw,file={disk},id=hd0",
        "-device", "virtio-blk-device,drive=hd0",
        "-drive", f"if=none,format=raw,file={disk},id=hd1",
        "-device", "virtio-blk-device,drive=hd1",
    ]
    base = f"http://127.0.0.1:{port}"
    with stderr.open("wb") as error_output:
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.DEVNULL,
                                   stderr=error_output, stdin=subprocess.PIPE)
        peer = None
        try:
            connect_deadline = time.monotonic() + 10
            while peer is None:
                try:
                    peer_sock = socket.create_connection(("127.0.0.1", peer_port), timeout=1)
                    peer = ClaimPeer(peer_sock, occupied, simultaneous, late)
                except ConnectionRefusedError:
                    if process.poll() is not None or time.monotonic() >= connect_deadline:
                        raise RuntimeError(stderr.read_text(errors="replace"))
                    time.sleep(0.1)
            assert process.stdin is not None
            process.stdin.write(b"cont\n")
            process.stdin.flush()
            if mode == "decline":
                deadline = time.monotonic() + 60
                while marker not in serial.read_text(errors="replace"):
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError(f"{name}: no DECLINE\n{serial.read_text(errors='replace')}")
                    time.sleep(0.1)
                assert "[dhcp] BOUND:" not in serial.read_text(errors="replace")
                assert any(len(f) > 284 and f[12:14] == b"\x08\0" and
                           f[23] == 17 and f[282:285] == b"\x35\1\4" for f in peer.frames)
                print(f"PASS {name}: real conflicting peer caused DHCPDECLINE, no bind")
                return
            deadline = time.monotonic() + 90
            while True:
                if process.poll() is not None:
                    raise RuntimeError(
                        f"{name}: QEMU exited: {stderr.read_text(errors='replace')}")
                try:
                    with urllib.request.urlopen(f"{base}/api/status", timeout=2) as response:
                        status_body = response.read().decode("utf-8")
                        if response.status != 200 or '"version"' not in status_body:
                            raise AssertionError(f"{name}: invalid status response")
                    break
                except (urllib.error.URLError, TimeoutError, ConnectionError):
                    if time.monotonic() >= deadline:
                        raise RuntimeError(
                            f"{name}: boot timed out\n"
                            f"{serial.read_text(errors='replace')[-6000:]}")
                    time.sleep(0.5)
            with socket.create_connection(("127.0.0.1", console_port), timeout=10) as sock:
                console = DebugConn(sock)
                console.read_until(b"debug> ")
                sock.sendall(b"unlock pios\r")
                console.read_until(b"pios> ")
                text = console.command(b"netcfg\r")
            for field in expected:
                if field not in text:
                    raise AssertionError(f"{name}: missing {field!r}: {text}")
            boot_log = serial.read_text(errors="replace")
            if marker not in boot_log:
                raise AssertionError(f"{name}: missing boot marker {marker!r}")
            if FIXTURE_KEY in boot_log or FIXTURE_KEY in text or FIXTURE_KEY in status_body:
                raise AssertionError(f"{name}: bootstrap key leaked")
            if mode == "cluster":
                assert boot_log.index("[cfg] full cluster pool census complete") < boot_log.index(marker)
                expected_pool = {socket.inet_aton(f"192.168.77.{i}") for i in range(1, 254)}
                assert set(peer.probes) == expected_pool, f"{name}: probes escaped explicit pool"
                for suffix in range(1, 254):
                    ip = socket.inet_aton(f"192.168.77.{suffix}")
                    assert peer.probes.get(ip, 0) >= 3, (
                        f"{name}: census missed {suffix}: count={peer.probes.get(ip, 0)} "
                        f"frames={len(peer.frames)} probes={len(peer.probes)} "
                        f"sample={[f.hex() for f in peer.frames[:3]]}")
                # Nothing may claim a source address before the entire census.
                seen: set[bytes] = set()
                for frame in peer.frames:
                    if len(frame) < 42 or frame[12:14] != b"\x08\x06":
                        continue
                    if frame[28:32] == b"\0" * 4:
                        seen.add(frame[38:42])
                    elif frame[22:28] == b"\x52\x54\0\x12\x34\x56":
                        assert len(seen) == 253, f"{name}: bound before full census"
                        break
            # All three real workers must survive deferred DHCP/cluster bind.
            for guest_port, path, markers in (
                (82, "/hello/config", ("PicoScript",)),
                (83, "/hello/config", ("native tiny fallback",)),
                (8090, "/hello/config", ('"capsule":"picoscript"',)),
            ):
                url = f"http://127.0.0.1:{capsule_ports[guest_port]}{path}"
                with urllib.request.urlopen(url, timeout=10) as response:
                    body = response.read().decode("utf-8")
                    if response.status != 200 or not all(m in body for m in markers):
                        raise AssertionError(f"{name}: capsule {guest_port}: {body}")
            print(f"PASS {name}: actual HTTP reachability and netcfg at {address}")
        finally:
            try:
                if peer:
                    peer.close()
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)
                if process.stdin:
                    process.stdin.close()
    disk.unlink()
    serial.unlink()
    stderr.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kernel", type=pathlib.Path,
                        default=ROOT / "build_qemu_full" / "PIOS_QEMU_FULL.BIN")
    parser.add_argument("--qemu", type=pathlib.Path,
                        default=pathlib.Path(r"C:\Program Files\qemu\qemu-system-aarch64.exe"))
    args = parser.parse_args()
    if not args.kernel.is_file() or not args.qemu.is_file():
        parser.error("build the QEMU full kernel and install QEMU before running")
    with tempfile.TemporaryDirectory(prefix="pios-boot-config-") as directory:
        folder = pathlib.Path(directory)
        cases = (
            ("missing", None, "dhcp", "[cfg] pios.ini unavailable; DHCP client selected"),
            ("network", NETWORK.encode("ascii"), "static", "[cfg] pios.ini accepted"),
            ("cluster", (CLUSTER_NETWORK + CLUSTER).encode("ascii"), "cluster",
             "[cfg] cluster address claimed (boot only); enrollment pending"),
            ("invalid-cluster", (NETWORK + CLUSTER.replace("seed3=", "seed4=")).encode("ascii"),
             "recovery", "[cfg] pios.ini rejected"),
        )
        for name, config, configured, marker in cases:
            check_boot(args.qemu.resolve(), args.kernel.resolve(), folder,
                       name, config, configured, marker)
        for name, simultaneous, late in (
            ("cluster-owner", False, False),
            ("cluster-simultaneous", True, False),
            ("cluster-final-collision", False, True),
        ):
            check_boot(args.qemu.resolve(), args.kernel.resolve(), folder,
                       name, (CLUSTER_NETWORK + CLUSTER).encode("ascii"), "cluster",
                       "[cfg] cluster address claimed (boot only); enrollment pending",
                       "192.168.77.4", simultaneous, late)
        check_boot(args.qemu.resolve(), args.kernel.resolve(), folder,
                   "dhcp-conflict", None, "decline",
                   "[dhcp] DECLINE sent for conflicting address", "192.168.0.15")
    print("PIOS boot configuration: all QEMU cases passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
