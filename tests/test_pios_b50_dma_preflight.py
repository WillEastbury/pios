import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import pios_b50_dma_preflight as p


class Board:
    def __init__(self):
        self.root = dict(p.EXPECTED)
        self.root[0x4008] = 0x48163480
        self.identities = dict(p.PATH + ((p.GPU, 0xE2128086),
                                       ((9, 0, 0), 0xE2128086),
                                       ((13, 0, 0), 0xE2128086)))
        self.commands = {bdf: 2 for bdf, _ in p.PATH}
        self.commands.update({p.GPU: 2, (9, 0, 0): 0, (13, 0, 0): 0})

    def peek(self, address):
        return self.root[address - p.ROOT]

    def read(self, bdf, reg):
        return self.identities[bdf] if reg == 0 else self.commands[bdf]

    def health(self):
        pass

    def record(self, **entry):
        pass


class Preflight(unittest.TestCase):
    def test_valid_is_not_dma_authorization(self):
        result = p.inspect(Board())
        for field in ("bus_master", "hardware_activation_allowed",
                      "dma_transfer_proven", "msi_delivery_proven"):
            self.assertFalse(result[field])

    def test_every_root_mismatch_rejects(self):
        for reg in p.EXPECTED:
            with self.subTest(reg=reg):
                board = Board()
                board.root[reg] ^= 1
                with self.assertRaises(RuntimeError):
                    p.inspect(board)
        board = Board()
        board.root[0x4008] ^= 1 << 27
        with self.assertRaises(RuntimeError):
            p.inspect(board)

    def test_identity_or_bme_rejects(self):
        baseline = Board()
        for bdf in baseline.identities:
            for kind in ("identity", "bme"):
                with self.subTest(bdf=bdf, kind=kind):
                    board = copy.deepcopy(baseline)
                    if kind == "identity":
                        board.identities[bdf] = 0xFFFFFFFF
                    else:
                        board.commands[bdf] |= 4
                    with self.assertRaises(RuntimeError):
                        p.inspect(board)


if __name__ == "__main__":
    unittest.main()
