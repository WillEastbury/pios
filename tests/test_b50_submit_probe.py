import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "submit_probe", Path(__file__).resolve().parents[1] /
    "tools" / "pios_b50_submit_probe.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def diag(state=0, failure=0, result=0, completion=0):
    return (f"b50 diag attached=1\n  submit state={state} response=F0000000 "
            f"failure={failure} result={result:08X} completion={completion:08X}"
            "\n engine head=00000000\n ct context=4/F0000000/0/40\n")


class Board:
    def __init__(self, final=None, present=True, drop=False):
        self.final = final or diag(5, result=42, completion=0xFEED0001)
        self.present = present
        self.drop = drop
        self.commands = []

    def status(self):
        return {"version": "test", "uptime": 1,
                "diag": {"error": 0}, "perf": {"nic_rx_wedge": 0}}

    def command(self, command):
        self.commands.append(command)
        if command == "pcie1 status":
            return f"pcie1 b50={int(self.present)}"
        if command == "b50 diag":
            return self.final if "b50 submit" in self.commands else diag()
        if command == "b50 submit":
            if self.drop:
                raise OSError("connection dropped")
            return "b50 bounded CCS0 store-only submission scheduled"
        return "revoked"


class SubmitProbeTests(unittest.TestCase):
    def test_absent_gpu_never_submits(self):
        board = Board(present=False)
        with self.assertRaisesRegex(probe.ProbeError, "B50 absent"):
            probe.probe(board)
        self.assertNotIn("b50 submit", board.commands)

    def test_exact_completion_and_revocation(self):
        board = Board()
        self.assertIn("FEED0001", probe.probe(board))
        self.assertEqual(board.commands[-1], "b50 revoke")

    def test_bad_result_or_failure_preserves_snapshot_and_revokes(self):
        for final in (diag(5, result=41, completion=0xFEED0001),
                      diag(0, failure=8)):
            board = Board(final=final)
            with self.assertRaises(probe.ProbeError) as error:
                probe.probe(board)
            self.assertIn("engine head=", str(error.exception))
            self.assertEqual(board.commands[-1], "b50 revoke")

    def test_lost_submit_response_still_revokes(self):
        board = Board(drop=True)
        with self.assertRaises(OSError):
            probe.probe(board)
        self.assertEqual(board.commands[-1], "b50 revoke")

    def test_wait_is_bounded(self):
        board = Board(final=diag(4))
        clock = iter((0, 1, 6))
        with self.assertRaisesRegex(probe.ProbeError, "deadline expired"):
            probe.probe(board, now=lambda: next(clock), sleep=lambda _: None)
        self.assertEqual(board.commands[-1], "b50 revoke")


if __name__ == "__main__":
    unittest.main()
