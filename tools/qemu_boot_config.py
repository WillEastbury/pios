#!/usr/bin/env python3
"""Verify FAT-root provisioning against the real PIOS kernel under QEMU."""
from __future__ import annotations

import argparse
import pathlib
import socket
import subprocess
import tempfile
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
    "seed1=192.168.0.210\nseed2=192.168.0.211\nseed3=192.168.0.212\n"
)


def check_boot(qemu: pathlib.Path, kernel: pathlib.Path, folder: pathlib.Path,
               name: str, config: bytes | None, mode: str,
               marker: str) -> None:
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
    elif mode == "dhcp":
        address = "192.168.0.15"
        expected = ("ip=192.168.0.15", "mask=255.255.255.0",
                    "gw=192.168.0.1", "dns=192.168.0.3")
    else:
        raise ValueError(f"unsupported boot mode: {mode}")
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        port = reserve.getsockname()[1]
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        console_port = reserve.getsockname()[1]
    network = (
        "user,id=n0,net=192.168.0.0/24,host=192.168.0.1,"
        f"hostfwd=tcp:127.0.0.1:{port}-{address}:80,"
        f"hostfwd=tcp:127.0.0.1:{console_port}-{address}:2323"
    )
    command = [
        str(qemu), "-M", "virt", "-cpu", "cortex-a76", "-smp", "4", "-m", "1G",
        "-display", "none", "-serial", f"file:{serial}",
        "-kernel", str(kernel), "-netdev", network,
        "-device", "virtio-net-device,netdev=n0",
        "-drive", f"if=none,format=raw,file={disk},id=hd0",
        "-device", "virtio-blk-device,drive=hd0",
        "-drive", f"if=none,format=raw,file={disk},id=hd1",
        "-device", "virtio-blk-device,drive=hd1",
    ]
    base = f"http://127.0.0.1:{port}"
    with stderr.open("wb") as error_output:
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.DEVNULL,
                                   stderr=error_output)
        try:
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
            print(f"PASS {name}: actual HTTP reachability and netcfg at {address}")
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
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
            ("cluster", (NETWORK + CLUSTER).encode("ascii"), "static",
             "[cfg] cluster provisioned; enrollment service not yet available"),
            ("invalid-cluster", (NETWORK + CLUSTER.replace("seed3=", "seed4=")).encode("ascii"),
             "recovery", "[cfg] pios.ini rejected"),
        )
        for name, config, configured, marker in cases:
            check_boot(args.qemu.resolve(), args.kernel.resolve(), folder,
                       name, config, configured, marker)
    print("PIOS boot configuration: all QEMU cases passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
