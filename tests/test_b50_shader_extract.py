import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location(
    "shader_extract", Path(__file__).resolve().parents[1] /
    "tools" / "extract_b50_shader.py")
extractor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extractor)


def image(kind=2, relocation_offset=4):
    names = b"\0.shstrtab\0.text.b50_w2a8\0.ze_info\0.strtab\0.symtab\0.rel.text.b50_w2a8\0"
    strings = b"\0__INTEL_PATCH_CROSS_THREAD_OFFSET_OFF_R0\0"
    payloads = [b"", names, bytes(16), b"version: '1.62'\n", strings,
                bytes(24) + struct.pack("<IBBHQQ", 1, 0x10, 0, 0, 0, 0),
                struct.pack("<QQ", relocation_offset, (1 << 32) | kind)]
    sections = [(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)]
    data = bytearray(64)
    for name, section_type, link, info, entry_size, payload in zip(
            (".shstrtab", ".text.b50_w2a8", ".ze_info", ".strtab",
             ".symtab", ".rel.text.b50_w2a8"),
            (3, 1, 1, 3, 2, 9), (0, 0, 0, 0, 4, 5),
            (0, 0, 0, 0, 0, 2), (0, 0, 0, 0, 24, 16), payloads[1:]):
        sections.append((names.index(name.encode()), section_type, 0, 0,
                         len(data), len(payload), link, info, 1, entry_size))
        data.extend(payload)
    table = len(data)
    for section in sections:
        data.extend(struct.pack("<IIQQQQIIQQ", *section))
    struct.pack_into("<16sHHIQQQIHHHHHH", data, 0,
                     b"\x7fELF\x02\x01\x01" + bytes(9), 1, 205, 1,
                     0, 0, table, 0, 64, 0, 0, 64, len(sections), 1)
    return bytes(data)


class ExtractionTests(unittest.TestCase):
    def test_tensor_metadata_drift_is_rejected(self):
        metadata = b"""version: '1.62'
grf_count: 128
simd_size: 32
inline_data_payload_size: 32
required_work_group_size: [ 16, 1, 1 ]
 - arg_type: arg_bypointer
   offset: 16
   size: 8
   arg_index: 0
   addrmode: stateless
 - arg_type: arg_bypointer
   offset: 24
   size: 8
   arg_index: 1
   addrmode: stateless
 - arg_type: arg_bypointer
   offset: 32
   size: 8
   arg_index: 2
   addrmode: stateless
 - arg_type: arg_byvalue
   offset: 40
   size: 4
   arg_index: 3
 - arg_type: arg_byvalue
   offset: 44
   size: 4
   arg_index: 4
 - arg_type: local_id
   offset: 0
   size: 192
"""
        extractor.validate_tensor_contract(metadata)
        for old, new in ((b"simd_size: 32", b"simd_size: 16"),
                         (b"offset: 24", b"offset: 28"),
                         (b"size: 192", b"size: 96"),
                         (b"stateless", b"bindful")):
            with self.assertRaises(ValueError):
                extractor.validate_tensor_contract(metadata.replace(old, new))

    def test_preserves_relocation_without_claiming_dispatch_ready(self):
        code, metadata, report = extractor.extract(image())
        self.assertEqual(len(code), 16)
        self.assertIn(b"1.62", metadata)
        self.assertEqual(report["relocations"][0]["offset"], 4)
        self.assertFalse(report["dispatch_ready"])

    def test_rejects_unknown_or_out_of_bounds_relocations(self):
        for data in (image(kind=99), image(relocation_offset=14)):
            with self.assertRaises(ValueError):
                extractor.extract(data)

    def test_rejects_truncated_section_table(self):
        for size in (0, 10, 63, len(image()) - 1):
            with self.assertRaises(ValueError):
                extractor.extract(image()[:size])


if __name__ == "__main__":
    unittest.main()
