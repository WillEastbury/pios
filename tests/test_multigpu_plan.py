import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from multigpu_plan import FORMAT_BITS, VERSION, PlanError, plan


def fixture():
    hosts = []
    layers = []
    for index, fmt in enumerate(("F16", "Q6_K")):
        hid = "host" + str(index)
        owner = dict(host=hid, host_generation=2, gpu="gpu0", gpu_generation=3)
        hosts.append(dict(id=hid, generation=2, available=True, gpus=[
            dict(id="gpu0", generation=3, available=True, capacity_bytes=100,
                 reserved=[], backends={"Xe2SIMD": FORMAT_BITS[fmt], "XMX": 0})]))
        layers.append(dict(id="layer" + str(index), backend="Xe2SIMD",
                           depends_on=[] if index == 0 else ["layer0"],
                           scratch_bytes=10, kv_bytes=20, state_bytes=5, input_bytes=8,
                           tensors=[dict(id="tensor" + str(index), generation=7,
                                         format=fmt, shape=[10], bytes=40,
                                         shards=[dict(id="shard" + str(index), generation=7,
                                                      tensor_offset=0, bytes=40, owner=owner)])]))
    return dict(version=VERSION, model_id="model", model_generation=7,
                hosts=hosts, layers=layers)


class PlacementTests(unittest.TestCase):
    def test_heterogeneous_local_deterministic_atomic(self):
        spec = fixture()
        original = copy.deepcopy(spec)
        result = plan(spec)
        self.assertEqual([p["owner"]["host"] for p in result["placements"]],
                         ["host0", "host1"])
        self.assertEqual(result["transfers"][0]["ideal_1gbe_ns"], 64)
        self.assertEqual(result, plan(spec))
        spec["hosts"].reverse()
        self.assertEqual(result, plan(spec))
        spec["hosts"].reverse()
        self.assertEqual(spec, original)
        self.assertFalse(result["activated"])

    def test_rejections_leave_input_untouched(self):
        def stale(s): s["layers"][1]["tensors"][0]["shards"][0]["owner"]["gpu_generation"] = 4
        def oom(s): s["hosts"][1]["gpus"][0]["capacity_bytes"] = 74
        def unavailable(s): s["hosts"][1]["available"] = False
        def unsupported(s): s["layers"][1]["backend"] = "XMX"
        def overlap(s):
            s["hosts"][0]["gpus"][0]["reserved"] = [
                dict(offset=0, bytes=20, owner="old", owner_generation=3),
                dict(offset=10, bytes=20, owner="old2", owner_generation=3)]
        def stale_reservation(s):
            s["hosts"][0]["gpus"][0]["reserved"] = [
                dict(offset=0, bytes=20, owner="old", owner_generation=2)]
        def overflow(s): s["layers"][1]["kv_bytes"] = 1 << 64
        def graph(s): s["layers"][1]["depends_on"] = []
        def unknown(s): s["layers"][0]["tensors"][0]["format"] = "Q4K"
        for mutate in (stale, oom, unavailable, unsupported, overlap,
                       stale_reservation, overflow, graph, unknown):
            with self.subTest(case=mutate.__name__):
                spec = fixture()
                mutate(spec)
                before = copy.deepcopy(spec)
                with self.assertRaises(PlanError):
                    plan(spec)
                self.assertEqual(before, spec)

    def test_exact_capacity_and_fragmented_disjoint_intervals(self):
        spec = fixture()
        gpu = spec["hosts"][0]["gpus"][0]
        gpu["reserved"] = [dict(offset=0, bytes=25, owner="existing", owner_generation=3)]
        result = plan(spec)
        self.assertEqual(result["placements"][0]["reservation"]["offset"], 25)
        gpu["reserved"][0]["offset"] = 50
        with self.assertRaises(PlanError):
            plan(spec)


if __name__ == "__main__":
    unittest.main()
