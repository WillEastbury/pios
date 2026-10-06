from io import StringIO, BytesIO
import json
from pathlib import Path
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import pios_kepler_vram as vram


class Board:
    def __init__(self, mismatch=False):
        self.selector = 0x20
        self.words = [i * 17 for i in range(vram.WORDS)]
        self.original = self.words[:]
        self.written = False
        self.mismatch = mismatch
        self.registers = {0x22438: 2, 0x2243c: 2, 0x22554: 0,
                          0x11020c: 1024, 0x11120c: 1024, 0x619f04: 1}

    def command(self, host, text):
        assert host == "unused"
        if text == "kepler probe":
            return "kepler probe ok\n posted=1 command=00000002"
        if text == "pcie1 aer":
            return "uncorr=00000000 corr=00000000"
        parts = text.split()
        addr = int(parts[1], 16)
        off = addr - vram.BAR0
        if parts[0] == "peek":
            if off == 0x1700:
                value = self.selector
            elif off >= 0x700000:
                assert self.selector == vram.SCRATCH >> 16
                index = (off - 0x700000) // 4
                value = self.words[index]
                if self.written and self.mismatch and index == 8:
                    value ^= 1
                    self.mismatch = False
            else:
                value = self.registers[off]
            return f"0x{addr:016X} = 0x{value:08X}"
        assert parts[0] == "poke"
        value = int(parts[2], 16)
        if off == 0x1700:
            self.selector = value
        else:
            index = (off - 0x700000) // 4
            assert 8 <= index < 24
            self.words[index] = value
            self.written = True
        return "OK:"


for mismatch in (False, True):
    board = Board(mismatch)
    health = BytesIO(json.dumps({"diag": {"error": 0},
                                "perf": {"nic_rx_wedge": 0}}).encode())
    with patch.object(vram, "terminal", side_effect=board.command), \
         patch.object(vram.urllib.request, "urlopen", return_value=health):
        if mismatch:
            try:
                vram.prove("unused", StringIO())
            except RuntimeError as exc:
                assert "mismatch" in str(exc)
            else:
                raise AssertionError("bad VRAM accepted")
        else:
            result = vram.prove("unused", StringIO())
            assert result["original_restored"] and not result["dma_enabled"]
    assert board.words == board.original
    assert board.selector == 0x20
print("Kepler VRAM: payload bounds, two patterns, guards and failure restoration passed")
