"""Offline, atomic placement contract. No device, file-payload or network access."""
import argparse
import json
from pathlib import Path

VERSION = "pios-placement-draft/1"
FORMATS = ("F32", "F16", "BF16", "I16", "I8", "I4", "Q4_0", "Q4_1",
           "Q4_K", "Q5_K", "Q6_K", "Q8_0", "Q8_K", "IQ4_XS", "MXFP4")
FORMAT_BITS = {name: 1 << index for index, name in enumerate(FORMATS)}
U64 = (1 << 64) - 1
MAX_BYTES = 1 << 50
MAX_INPUT = 4 << 20


class PlanError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise PlanError(message)


def number(value, maximum=U64):
    require(type(value) is int and 0 <= value <= maximum, "invalid integer")
    return value


def identity(value):
    require(isinstance(value, str) and 0 < len(value) <= 128
            and all(c.isalnum() or c in "._-" for c in value), "invalid identity")
    return value


def unique(items, key):
    require(len({item[key] for item in items}) == len(items), "duplicate identity")


def plan(spec):
    """Return one complete proposal or raise; never mutate caller reservations."""
    require(spec["version"] == VERSION, "version mismatch")
    model = identity(spec["model_id"])
    model_gen = number(spec["model_generation"])
    require(model_gen > 0, "zero generation")
    hosts, layers = spec["hosts"], spec["layers"]
    require(0 < len(hosts) <= 16 and 0 < len(layers) <= 256, "table bounds")
    unique(hosts, "id")
    unique(layers, "id")
    devices, shard_ids, placements, transfers = [], set(), [], []
    tensor_ids = set()
    for host in hosts:
        hid = identity(host["id"])
        hgen = number(host["generation"])
        require(hgen > 0 and type(host["available"]) is bool, "host generation/status")
        require(0 < len(host["gpus"]) <= 16, "GPU table bounds")
        unique(host["gpus"], "id")
        for gpu in host["gpus"]:
            gid = identity(gpu["id"])
            gen = number(gpu["generation"])
            require(gen > 0 and type(gpu["available"]) is bool, "GPU generation/status")
            capacity = number(gpu["capacity_bytes"], MAX_BYTES)
            intervals = sorted(gpu["reserved"], key=lambda r: r["offset"])
            require(len(intervals) <= 256, "reservation bounds")
            end = 0
            for interval in intervals:
                start = number(interval["offset"], MAX_BYTES)
                size = number(interval["bytes"], MAX_BYTES)
                require(size > 0 and start >= end and size <= capacity - start,
                        "overlap or reservation overflow")
                require(number(interval["owner_generation"]) == gen, "stale reservation")
                identity(interval["owner"])
                end = start + size
            caps = gpu["backends"]
            require(set(caps) <= {"Xe2SIMD", "XMX"}, "unknown backend")
            for mask in caps.values():
                number(mask, (1 << len(FORMATS)) - 1)
            devices.append(dict(host=hid, host_generation=hgen, gpu=gid,
                                gpu_generation=gen, capacity=capacity,
                                available=host["available"] and gpu["available"],
                                intervals=[dict(r) for r in intervals], caps=caps))
    devices.sort(key=lambda d: (d["host"], d["gpu"]))
    previous = None
    total_tensors = 0
    for index, layer in enumerate(layers):
        lid = identity(layer["id"])
        require(layer["depends_on"] == ([] if index == 0 else [layers[index - 1]["id"]]),
                "only explicit linear layer graphs supported")
        backend = layer["backend"]
        require(backend in {"Xe2SIMD", "XMX"}, "unsupported backend")
        number(layer["input_bytes"], MAX_BYTES)
        costs = {k: number(layer[k + "_bytes"], MAX_BYTES)
                 for k in ("scratch", "kv", "state")}
        weights = 0
        tensors = layer["tensors"]
        total_tensors += len(tensors)
        require(0 < len(tensors) and total_tensors <= 4096, "tensor bounds")
        mask = 0
        for tensor in tensors:
            tid = identity(tensor["id"])
            require(tid not in tensor_ids, "duplicate tensor")
            tensor_ids.add(tid)
            require(tensor["format"] in FORMAT_BITS, "unsupported storage format")
            mask |= FORMAT_BITS[tensor["format"]]
            require(number(tensor["generation"]) == model_gen, "stale tensor generation")
            shape = tensor["shape"]
            require(0 < len(shape) <= 8 and all(number(n, 1 << 32) > 0 for n in shape),
                    "invalid shape")
            elements = 1
            for dimension in shape:
                require(elements <= U64 // dimension, "shape overflow")
                elements *= dimension
            shards = tensor["shards"]
            require(0 < len(shards) <= 256, "shard bounds")
            end = 0
            for shard in sorted(shards, key=lambda s: s["tensor_offset"]):
                sid = identity(shard["id"])
                require(sid not in shard_ids, "duplicate shard")
                shard_ids.add(sid)
                size = number(shard["bytes"], MAX_BYTES)
                require(size > 0 and shard["tensor_offset"] == end, "shard gap/overlap")
                require(number(shard["generation"]) == model_gen, "stale shard generation")
                source = shard["owner"]
                matches = [d for d in devices if d["host"] == source["host"]
                           and d["gpu"] == source["gpu"]]
                require(len(matches) == 1 and matches[0]["available"], "unavailable shard owner")
                require(number(source["host_generation"]) == matches[0]["host_generation"]
                        and number(source["gpu_generation"]) == matches[0]["gpu_generation"],
                        "stale shard owner")
                end += size
            require(end == number(tensor["bytes"], MAX_BYTES), "tensor extent mismatch")
            weights += end
        costs["weights"] = weights
        size = sum(costs.values())
        require(0 < size <= MAX_BYTES, "layer reservation overflow")
        candidates = []
        for d in devices:
            if not d["available"] or d["caps"].get(backend, 0) & mask != mask:
                continue
            offset = 0
            for r in d["intervals"]:
                if size <= r["offset"] - offset:
                    break
                offset = r["offset"] + r["bytes"]
            if size > d["capacity"] - offset:
                continue
            remote = sum(s["bytes"] for t in tensors for s in t["shards"]
                         if s["owner"]["host"] != d["host"])
            activation = (number(layer["input_bytes"], MAX_BYTES)
                          if previous and previous["host"] != d["host"] else 0)
            candidates.append((remote + activation, d["host"], d["gpu"], offset, d))
        require(candidates, "no compatible capacity for layer " + lid)
        _, _, _, offset, chosen = min(candidates, key=lambda c: c[:4])
        owner = {k: chosen[k] for k in ("host", "host_generation", "gpu", "gpu_generation")}
        reservation = dict(owner=lid, owner_generation=chosen["gpu_generation"],
                           offset=offset, bytes=size)
        chosen["intervals"].append(reservation)
        chosen["intervals"].sort(key=lambda r: r["offset"])
        placements.append(dict(layer=lid, owner=owner, backend=backend,
                               format_mask=mask, reservation=reservation, breakdown=costs))
        for t in tensors:
            for s in t["shards"]:
                if s["owner"]["host"] != chosen["host"]:
                    transfers.append(dict(kind="weight", tensor=t["id"], shard=s["id"],
                                          generation=model_gen, source=s["owner"],
                                          destination=owner, bytes=s["bytes"]))
        if previous and previous["host"] != chosen["host"]:
            transfers.append(dict(kind="activation", layer=lid, source=previous,
                                  destination=owner, bytes=number(layer["input_bytes"], MAX_BYTES)))
        previous = owner
    for transfer in transfers:
        transfer["ideal_1gbe_ns"] = transfer["bytes"] * 8
    return dict(version=VERSION, activated=False, model_id=model,
                model_generation=model_gen, placements=placements, transfers=transfers,
                transfer_assumption="1GbE ideal serialization only; no latency/overhead guarantee")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        with args.input.open("rb") as stream:
            raw = stream.read(MAX_INPUT + 1)
        require(len(raw) <= MAX_INPUT, "input budget")
        def pairs(entries):
            result = {}
            for key, value in entries:
                require(key not in result, "duplicate JSON key")
                result[key] = value
            return result
        result = plan(json.loads(raw, object_pairs_hook=pairs))
    except (ValueError, KeyError, TypeError, OverflowError) as error:
        parser.exit(1, "plan rejected: " + str(error) + "\n")
    args.output.write_text(json.dumps(result, sort_keys=True, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
