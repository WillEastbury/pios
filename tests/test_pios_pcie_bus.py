"""Physical-tree config model: sibling routing collisions reject access."""
from pathlib import Path
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import pios_pcie_bus as bus


class Node:
    def __init__(self, identity, children=None, command=0):
        self.identity, self.children, self.command = identity, children, command
        self.buses = 0


class Config:
    def __init__(self):
        # Each B50 branch has two bridge hops below the PLX downstream port.
        self.gpus = [Node(0xE2128086) for _ in range(3)]
        self.ports = [Node(bus.PLX_ID, {0: Node(0xE2FF8086, {0: Node(
            0xE2FF8086, {0: gpu})})}) for gpu in self.gpus]
        self.ports.append(Node(bus.PLX_ID, {}))
        self.root = Node(bus.PLX_ID, {8 * 8: self.ports[0], 9 * 8: self.ports[1],
                                     16 * 8: self.ports[2], 17 * 8: self.ports[3]})
        self.root.buses = 0x080201
        for i, port in enumerate(self.ports):
            port.buses = 2 | ((3 + i) << 8) | (8 << 16)
        self.writes = []
        self.reads = 0
        self.drop_write = False

    def node(self, bdf):
        target, dev, fn = bdf
        def descend(nodes, current):
            if current == target:
                return nodes.get(dev * 8 + fn)
            routes = [n for n in nodes.values() if n.children is not None and
                      0 < ((n.buses >> 8) & 255) <= target <= ((n.buses >> 16) & 255)]
            assert len(routes) <= 1, "overlapping routes used"
            if not routes:
                return None
            n = routes[0]
            return descend(n.children, (n.buses >> 8) & 255)
        return descend({0: self.root}, 1)

    def read(self, bdf, reg):
        self.reads += 1
        assert self.reads < 20000
        node = self.node(bdf)
        if node is None:
            return 0xFFFFFFFF
        return {0: node.identity, 4: node.command,
                8: 0x06040000 if node.children is not None else 0x03020000,
                12: 0x10000 if node.children is not None else 0,
                24: node.buses}[reg]

    def write(self, bdf, reg, value):
        assert reg == 24
        node = self.node(bdf)
        assert node and node.children is not None and not node.command & 7
        self.writes.append((node, value))
        if self.drop_write:
            self.drop_write = False
            return
        node.buses = value


model = Config()
result = bus.configure(model)
assert result["b50_count"] == 3 and result["buses_used"] > 8
assert all(n.command == 0 for n in model.gpus + model.ports)
for left, right in zip(model.ports, model.ports[1:]):
    assert ((left.buses >> 16) & 255) < ((right.buses >> 8) & 255)
for fail in ("active", "capacity", "readback"):
    model = Config()
    if fail == "active":
        model.root.command = 2
    elif fail == "readback":
        model.drop_write = True
    with patch.object(bus, "MAX_BUS", 5 if fail == "capacity" else 63):
        try:
            bus.configure(model)
        except bus.BusError:
            pass
        else:
            raise AssertionError("invalid fabric accepted")
    if fail == "active":
        assert not model.writes
    else:
        assert model.root.buses & 0xFFFFFF == 0
print("PCIe bus assignment: nested three-GPU tree, non-overlap, refusal and quarantine passed")
