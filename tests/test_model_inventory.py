import io
import json
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import model_inventory as m


def string(s):
    b = s.encode()
    return struct.pack("<Q", len(b)) + b


def gguf_header(types=(0, 1, 2, 12, 8), version=3):
    b = b"GGUF" + struct.pack("<IQQ", version, len(types), 0)
    for i, t in enumerate(types):
        b += string(str(i)) + struct.pack("<IQIQ", 1, 256, t, i * 2048)
    return b


def safe_header(desc):
    b = json.dumps(desc).encode()
    return struct.pack("<Q", len(b)) + b


class HeaderTests(unittest.TestCase):
    def test_gguf_types_and_no_payload_read(self):
        for v in (2, 3):
            b = gguf_header(version=v)
            f = io.BytesIO(b)
            result = m.gguf(m.Reader(f, 100_000))
            self.assertEqual(result["tensor_types"],
                             {"F32": 1, "F16": 1, "Q4_0": 1, "Q4_K": 1, "Q8_0": 1})
            self.assertEqual(f.tell(), len(b))

    def test_safetensors_types(self):
        b = safe_header({"a": {"dtype": "I16", "shape": [4], "data_offsets": [0, 8]},
                         "b": {"dtype": "F16", "shape": [2], "data_offsets": [8, 12]}})
        f = io.BytesIO(b)
        self.assertEqual(m.safetensors(m.Reader(f, len(b) + 12))["tensor_types"],
                         {"I16": 1, "F16": 1})
        self.assertEqual(f.tell(), len(b))

    def test_truncation_and_version(self):
        b = gguf_header()
        for n in (0, 3, 20, len(b) - 1):
            with self.assertRaises(m.Invalid):
                m.gguf(m.Reader(io.BytesIO(b[:n]), n))
        with self.assertRaises(m.Invalid):
            m.gguf(m.Reader(io.BytesIO(gguf_header(version=1)), 100_000))

    def test_overlap_and_file_range(self):
        for offsets in ([4, 12], [8, 100]):
            b = safe_header({"a": {"dtype": "I8", "shape": [8], "data_offsets": [0, 8]},
                             "b": {"dtype": "I8", "shape": [offsets[1] - offsets[0]],
                                   "data_offsets": offsets}})
            with self.assertRaises(m.Invalid):
                m.safetensors(m.Reader(io.BytesIO(b), len(b) + 16))
        b = gguf_header((0, 0))
        # Second descriptor shares first descriptor's offset.
        b = b[:-8] + struct.pack("<Q", 0)
        with self.assertRaises(m.Invalid):
            m.gguf(m.Reader(io.BytesIO(b), 100_000))

    def test_budgets_duplicates_and_overflow(self):
        b = struct.pack("<Q", m.MAX_HEADER)
        with self.assertRaises(m.Invalid):
            m.safetensors(m.Reader(io.BytesIO(b), m.MAX_HEADER + 8))
        for text in ('{"a":1,"a":2}', "[" * 17 + "]" * 17, '{"a":'):
            raw = text.encode()
            b = struct.pack("<Q", len(raw)) + raw
            with self.assertRaises((m.Invalid, json.JSONDecodeError)):
                m.safetensors(m.Reader(io.BytesIO(b), len(b)))
        with self.assertRaises(m.Invalid):
            m.product([1 << 63, 4])

    def test_unknown_type_is_not_generic_int4(self):
        b = gguf_header((999,))
        result = m.gguf(m.Reader(io.BytesIO(b), 100_000))
        self.assertEqual(result["tensor_types"], {"UNKNOWN_GGML_999": 1})
        self.assertIn("incomplete", result["range_validation"])

    def test_quant_types_alignment_and_unknown_range(self):
        b = gguf_header((23, 39, 25))
        result = m.gguf(m.Reader(io.BytesIO(b), 100_000))
        self.assertEqual(result["tensor_types"], {"IQ4_XS": 1, "MXFP4": 1, "I16": 1})
        b = gguf_header((2,))
        for offset in (1, 100_000):
            altered = b[:-8] + struct.pack("<Q", offset)
            with self.assertRaises(m.Invalid):
                m.gguf(m.Reader(io.BytesIO(altered), 100_000))
        b = gguf_header((999,))
        altered = b[:-8] + struct.pack("<Q", 99_968)
        with self.assertRaises(m.Invalid):
            m.gguf(m.Reader(io.BytesIO(altered), 100_000))


if __name__ == "__main__":
    unittest.main()
