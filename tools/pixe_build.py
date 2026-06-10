#!/usr/bin/env python3
"""pixe_build.py - compile PicoScript and upload it to PIOS as a picowal card.

This realises "compiler for the platform": it compiles a PicoScript source file
to picovm bytecode (via the willeastbury/picoscript reference toolchain) and
streams the bytecode into a PIOS picowal card over the HTTP terminal using the
kernel `pixe put` command, then optionally executes it on-board with `pixe card`.

The card format is raw little-endian bytecode (4 bytes per 32-bit word), which is
exactly what the kernel `pixe card <card> <record>` loader reconstructs and runs.

Examples:
  # Compile examples/sum.pc and upload to card 100 record 1, then run it:
  python tools/pixe_build.py ../picoscript/examples/sum.pc --card 100 --record 1 --run

  # Upload an already-compiled hex bytecode file (one 8-hex-digit word per line):
  python tools/pixe_build.py sum.hex --from-hex --card 100 --record 1 --run

  # Just compile and print the bytecode (no upload):
  python tools/pixe_build.py ../picoscript/examples/sum.pc --emit-only
"""
import argparse
import os
import re
import subprocess
import sys
import urllib.parse
import urllib.request

# The kernel HTTP terminal `cmd` buffer is 128 bytes. The `pixe put` prefix
# ("pixe put <card> <record> <offset> ") is at most ~26 chars and each appended
# word costs 9 chars ("AABBCCDD "). 8 words/chunk stays well under the limit.
DEFAULT_WORDS_PER_CHUNK = 8


def find_compiler(explicit):
    """Locate picoscript_build.py (the reference compiler)."""
    candidates = []
    if explicit:
        candidates.append(explicit)
    env = os.environ.get("PICOSCRIPT_HOME")
    if env:
        candidates.append(os.path.join(env, "picoscript_build.py"))
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, "..", "..", "picoscript", "picoscript_build.py"))
    candidates.append(os.path.join(here, "..", "picoscript", "picoscript_build.py"))
    for c in candidates:
        if c and os.path.isfile(c):
            return os.path.abspath(c)
    return None


def parse_hex_words(text):
    """Parse whitespace/line separated 8-hex-digit words into a list of ints."""
    words = []
    for tok in re.split(r"\s+", text.strip()):
        if not tok:
            continue
        if tok.lower().startswith("0x"):
            tok = tok[2:]
        if not re.fullmatch(r"[0-9a-fA-F]{1,8}", tok):
            raise ValueError("not a hex word: %r" % tok)
        words.append(int(tok, 16) & 0xFFFFFFFF)
    return words


def compile_source(compiler, source, lang=None):
    """Compile a PicoScript source file to a list of bytecode words."""
    cmd = [sys.executable, compiler, "emit", source, "--as", "bytecode", "--hex"]
    if lang:
        cmd += ["--lang", lang]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        sys.stderr.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        raise SystemExit("compiler failed (exit %d)" % proc.returncode)
    return parse_hex_words(proc.stdout)


def term(host, command, timeout):
    """Run one PIOS terminal command over HTTP and return the text response."""
    url = "http://%s/api/terminal?cmd=%s" % (host, urllib.parse.quote(command))
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read().decode("utf-8", "replace").strip()


def upload(host, card, record, words, per_chunk, timeout):
    """Stream bytecode words to a picowal card via chunked `pixe put`."""
    total = len(words)
    off = 0
    last = ""
    while off < total:
        chunk = words[off:off + per_chunk]
        hexwords = " ".join("%08x" % w for w in chunk)
        command = "pixe put %d %d %d %s" % (card, record, off, hexwords)
        if len(command) > 127:
            raise SystemExit("chunk command too long (%d bytes); lower --words-per-chunk"
                             % len(command))
        last = term(host, command, timeout)
        if "ERR" in last or "usage" in last:
            raise SystemExit("upload failed at offset %d: %s" % (off, last))
        print("  [%3d/%3d] %s" % (off + len(chunk), total, last))
        off += len(chunk)
    return last


def main():
    ap = argparse.ArgumentParser(description="Compile PicoScript and upload to a PIOS picowal card.")
    ap.add_argument("source", help="PicoScript source file (or hex bytecode with --from-hex)")
    ap.add_argument("--from-hex", action="store_true",
                    help="treat source as a hex bytecode file (one word per line) instead of compiling")
    ap.add_argument("--lang", choices=["c", "basic", "python", "english", "v1"],
                    help="force the PicoScript frontend language")
    ap.add_argument("--compiler", help="path to picoscript_build.py (else auto-detect / $PICOSCRIPT_HOME)")
    ap.add_argument("--host", default="192.168.0.200", help="PIOS host (default 192.168.0.200)")
    ap.add_argument("--card", type=int, help="destination picowal card (0..1023)")
    ap.add_argument("--record", type=int, help="destination picowal record")
    ap.add_argument("--run", action="store_true", help="execute on-board with `pixe card` after upload")
    ap.add_argument("--emit-only", action="store_true", help="compile and print bytecode; do not upload")
    ap.add_argument("--words-per-chunk", type=int, default=DEFAULT_WORDS_PER_CHUNK,
                    help="bytecode words per `pixe put` chunk (default %d)" % DEFAULT_WORDS_PER_CHUNK)
    ap.add_argument("--timeout", type=float, default=10.0, help="HTTP timeout seconds")
    args = ap.parse_args()

    if args.from_hex:
        with open(args.source, "r") as f:
            words = parse_hex_words(f.read())
        print("loaded %d bytecode words from %s" % (len(words), args.source))
    else:
        compiler = find_compiler(args.compiler)
        if not compiler:
            raise SystemExit("could not find picoscript_build.py; pass --compiler or set $PICOSCRIPT_HOME")
        words = compile_source(compiler, args.source, args.lang)
        print("compiled %s -> %d bytecode words (via %s)"
              % (args.source, len(words), os.path.basename(compiler)))

    if not words:
        raise SystemExit("no bytecode produced")

    if args.emit_only:
        for w in words:
            print("%08x" % w)
        return

    if args.card is None or args.record is None:
        raise SystemExit("--card and --record are required to upload (or use --emit-only)")
    if not (0 <= args.card <= 1023):
        raise SystemExit("--card must be 0..1023")
    if args.words_per_chunk < 1:
        raise SystemExit("--words-per-chunk must be >= 1")

    print("uploading %d words to %s card=%d record=%d (%d words/chunk)"
          % (len(words), args.host, args.card, args.record, args.words_per_chunk))
    upload(args.host, args.card, args.record, words, args.words_per_chunk, args.timeout)
    print("upload complete: card=%d record=%d words=%d" % (args.card, args.record, len(words)))

    if args.run:
        result = term(args.host, "pixe card %d %d" % (args.card, args.record), args.timeout)
        print("run: %s" % result)


if __name__ == "__main__":
    main()
