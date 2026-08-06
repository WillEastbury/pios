#!/usr/bin/env python3
"""pixe_endpoint_ref.py - reference oracle for PIOS EL0 "pix endpoint" programs.

Runs a PicoScript bytecode program on the off-board reference VM
(picoscript_vm.PicoVM) with the Context.* / Io.* host hooks wired exactly the way
PIOS src/pixe_host.c wires them:

  * Context string fields (verb/path/host/headers/querystring/body) materialize
    into the VM arena via HostApi._str_span and return a span handle.
  * Context scalar fields (user/permissions/port/requestid) return integers.
  * Io.Write / Io.WriteByte append bytes to the response output.

so the printed status / output bytes / register handles are the byte-parity target
the on-board `pixe endpoint` command must reproduce.

Usage:
  # default echo endpoint against the canned POST /api/sum?n=10 ... body "hello"
  python tools/pixe_endpoint_ref.py

  # arbitrary program + request fields
  python tools/pixe_endpoint_ref.py --hex 000070E0,00007071,...,C0000000 \
      --verb POST --path /api/sum --query n=10 --body hello \
      --host pios.local --user 0 --perms 0x1f --reqid 1
"""
import argparse
import os
import sys

# Default: resolve the reference compiler/VM next to PIOS or via $PICOSCRIPT_HOME.
_DEF = os.environ.get("PICOSCRIPT_HOME") or os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "picoscript")

# The embedded echo endpoint (matches pixe_host.c pixe_echo_endpoint):
#   Context.GetVerb->R0; Io.Write(R0); Context.GetBody->R1; Io.Write(R1);
#   Net.Status(200); Flow.Return
ECHO = [0x000070E0, 0x00007071, 0x010070E9, 0x00107071, 0x000080C8, 0xC0000000]


def parse_words(s):
    out = []
    for tok in s.replace(",", " ").split():
        out.append(int(tok, 16) & 0xFFFFFFFF)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--compiler", default=_DEF, help="path to the picoscript repo")
    ap.add_argument("--hex", help="program words as comma/space separated hex (default: echo)")
    ap.add_argument("--verb", default="POST")
    ap.add_argument("--path", default="/api/sum")
    ap.add_argument("--query", default="n=10")
    ap.add_argument("--headers", default="Host: pios.local\r\nContent-Type: text/plain\r\nContent-Length: 5\r\n")
    ap.add_argument("--host", dest="hosthdr", default="pios.local")
    ap.add_argument("--body", default="hello")
    ap.add_argument("--user", type=lambda v: int(v, 0), default=0)
    ap.add_argument("--perms", type=lambda v: int(v, 0), default=0x1f)
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--reqid", type=int, default=1)
    args = ap.parse_args()

    sys.path.insert(0, args.compiler)
    try:
        from picoscript_vm import PicoVM, HostApi
    except Exception as e:  # pragma: no cover
        print(f"error: cannot import picoscript_vm from {args.compiler}: {e}", file=sys.stderr)
        return 2

    host = HostApi()
    strs = {
        "GetVerb": args.verb, "GetPath": args.path, "GetHost": args.hosthdr,
        "GetHeaders": args.headers, "GetQueryString": args.query, "GetBody": args.body,
        "GetRemoteAddr": "",
    }
    ints = {
        "GetUser": args.user, "GetPermissions": args.perms,
        "GetPort": args.port, "GetRequestId": args.reqid,
    }
    for m, v in strs.items():
        host.register("Context", m,
                      lambda vm, rd, rs1, rs2, imm, _v=v: vm.regs.__setitem__(rd, host._str_span(vm, _v)))
    for m, v in ints.items():
        host.register("Context", m,
                      lambda vm, rd, rs1, rs2, imm, _v=v: vm.regs.__setitem__(rd, _v & 0xFFFFFFFF))

    prog = parse_words(args.hex) if args.hex else ECHO
    vm = PicoVM(host=host)
    vm.run(prog)
    out = b"".join(vm.output)
    print("pixe endpoint reference (parity target):")
    print(f"  words  = {len(prog)}")
    print(f"  steps  = {vm.steps}")
    print(f"  status = {vm.http_status}")
    print(f"  regs   = {vm.regs[:8]}")
    print(f"  out    = {out!r} (len {len(out)})")
    print(f"  outhex = {out.hex()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
