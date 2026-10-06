"""Bounded GK107 POST interpreter with injected device operations.

No hardware backend is selected by this module. Callers must implement
read/write, VGA, GPIO and I2C operations, and supply a monotonic clock.
Hardware semantics are derived from Nouveau nvkm/subdev/bios/init.c and
ramcfg.c; this is not an x86 or EFI option-ROM executor.
"""
import time

from pios_kepler_rom import inspect_scripts, validate_rom

MASK32 = 0xFFFFFFFF


class PostError(ValueError):
    pass


class Post:
    MAX_STEPS = 16384
    MAX_DEPTH = 16
    MAX_SECONDS = 120
    SUPPORTED = {
        "CONDITION", "CONDITION_TIME", "COPY_NV_REG", "COPY_ZM_REG", "CR",
        "DONE", "END_REPEAT", "GPIO", "I2C_BYTE", "IO", "IO_CONDITION",
        "MACRO", "NOT", "NV_REG", "RAM_RESTRICT_ZM_REG_GROUP", "REPEAT",
        "RESET_BEGUN", "RESET_END", "RESUME", "SUB_DIRECT", "TIME", "XLAT",
        "ZM_CR", "ZM_MASK_ADD", "ZM_REG", "ZM_REG_GROUP", "ZM_REG_SEQUENCE",
    }

    def __init__(self, rom, bus, clock=time.monotonic):
        report = validate_rom(rom)
        inventory = inspect_scripts(rom)
        missing = set(inventory["opcodes"]) - self.SUPPORTED
        if missing:
            raise PostError(f"unsupported POST operations: {sorted(missing)}")
        self.rom = bytes(rom[:report["images"][0]["bytes"]])
        self.bus = bus
        self.clock = clock
        self.records = {r["offset"]: r for r in inventory["instructions"]}
        self.roots = [int(p, 16) for p in inventory["entrypoints"]]
        self.entries = {e["id"]: e for e in report["bit_entries"]}
        self.groups = inventory["ram_configurations"]
        self.steps = 0
        self.writes = 0
        self.pc = 0
        self.enabled = True
        self.ram_index = None
        self.deadline = 0
        self.started = False
        self.complete = False
        self.error = None
        self.ram_choices = {}
        self.preflight()

    def bytes(self, offset, length):
        if offset < 0 or length < 0 or offset + length > len(self.rom):
            raise PostError(f"ROM range {offset:#x}+{length:#x} is invalid")
        return self.rom[offset:offset + length]

    def byte(self, offset):
        return self.bytes(offset, 1)[0]

    def word(self, offset):
        return int.from_bytes(self.bytes(offset, 2), "little")

    def dword(self, offset):
        return int.from_bytes(self.bytes(offset, 4), "little")

    def table(self, field):
        entry = self.entries["I"]
        if field + 2 > entry["bytes"]:
            raise PostError("short BIT init directory")
        result = self.word(entry["offset"] + field)
        if not result:
            raise PostError(f"missing BIT init table {field:#x}")
        return result

    @staticmethod
    def register(addr):
        # This first backend has no display-head/output binding for encoded
        # high address bits. PCI Command aliases cannot become a DMA bypass.
        if addr & ~0x00FFFFFC or addr in (0x1804, 0x88004):
            raise PostError(f"unsupported or protected register {addr:#x}")
        return addr

    def preflight(self):
        """Resolve operand/table bounds before the first device operation."""
        for pc, record in self.records.items():
            op = record["opcode"]
            self.bytes(pc, record["bytes"])
            if op in (0x6E, 0x7A, 0x91, 0x97):
                self.register(self.dword(pc + 1))
            elif op == 0x58:
                base = self.dword(pc + 1)
                for i in range(self.byte(pc + 5)):
                    self.register(base + 4 * i)
            elif op == 0x8F:
                if not 0 < self.groups <= 32:
                    raise PostError("missing RAM configuration groups")
                base, stride = self.dword(pc + 1), self.byte(pc + 5)
                for i in range(self.byte(pc + 6)):
                    self.register(base + stride * i)
                self.ram_layout()
            elif op == 0x5F:
                self.register(self.dword(pc + 1))
                self.register(self.dword(pc + 14))
                self.shift(0, self.byte(pc + 5))
            elif op == 0x90:
                self.register(self.dword(pc + 1))
                self.register(self.dword(pc + 5))
            elif op in (0x75, 0x56):
                ptr = self.table(6) + self.byte(pc + 1) * 12
                self.bytes(ptr, 12)
                self.register(self.dword(ptr))
            elif op == 0x76:
                ptr = self.table(8) + self.byte(pc + 1) * 5
                self.bytes(ptr, 5)
                if self.word(ptr) not in (0x3C4, 0x3CE, 0x3D4):
                    raise PostError("unsupported indexed VGA port")
            elif op == 0x6F:
                ptr = self.table(4) + self.byte(pc + 1) * 8
                self.bytes(ptr, 8)
                self.register(self.dword(ptr))
            elif op == 0x96:
                self.register(self.dword(pc + 1))
                self.register(self.dword(pc + 8))
                self.shift(0, self.byte(pc + 5))
                if self.byte(pc + 16) >= 32:
                    raise PostError("invalid XLAT shift")
                ptr = self.word(self.table(16) + self.byte(pc + 7) * 2)
                self.bytes(ptr, self.byte(pc + 6) + 1)
            elif op == 0x33 and self.byte(pc + 1) == 0:
                raise PostError("zero-count repeat is unsupported")
            elif op == 0x69 and self.word(pc + 1) != 0x3C3:
                raise PostError("unsupported direct VGA port")
            elif op == 0x4C:
                if self.byte(pc + 1) != 0x80 or self.byte(pc + 2) != 0x98:
                    raise PostError("I2C opcode outside reviewed board profile")
                for i in range(self.byte(pc + 3)):
                    if self.byte(pc + 4 + i * 3) != 9:
                        raise PostError("I2C register outside reviewed board profile")
        # Backends must acknowledge every side-effect family up front.
        self.bus.preflight(set(r["name"] for r in self.records.values()))

    def tick(self):
        if self.clock() >= self.deadline:
            raise PostError(f"POST deadline at {self.pc:#x}")

    def read(self, addr):
        self.tick()
        addr = self.register(addr)
        if not self.enabled:
            return 0
        result = self.bus.read32(addr)
        if not isinstance(result, int) or not 0 <= result <= MASK32:
            raise PostError("invalid MMIO read result")
        return result

    def write(self, addr, value):
        self.tick()
        addr = self.register(addr)
        if self.enabled:
            self.bus.write32(addr, value & MASK32)
            self.writes += 1

    def mask(self, addr, keep, value):
        if self.enabled:
            self.write(addr, (self.read(addr) & keep) | value)

    @staticmethod
    def shift(value, amount):
        shift = amount if amount < 128 else 256 - amount
        if shift >= 32:
            raise PostError("invalid register shift")
        return (value >> shift) if amount < 128 else (value << shift) & MASK32

    def condition(self, index):
        ptr = self.table(6) + index * 12
        return self.read(self.dword(ptr)) & self.dword(ptr + 4) == self.dword(ptr + 8)

    def ram_layout(self):
        if self.ram_choices:
            return
        m = self.entries.get("M")
        if not m or m["version"] != 2 or m["bytes"] < 7:
            raise PostError("unsupported BIT M layout")
        ptr = self.word(m["offset"] + 3)
        if not ptr or self.byte(ptr) != 0x10:
            raise PostError("unsupported RAM strap table")
        header, stride, count = self.byte(ptr + 1), self.byte(ptr + 2), self.byte(ptr + 3)
        if header < 7 or stride < 2 or count > 32 or self.byte(ptr + 4) != 0:
            raise PostError("invalid RAM strap table")
        self.bytes(ptr, header + count * stride)
        for i in range(count):
            entry = ptr + header + i * stride
            strap = self.byte(entry) >> 4
            group = self.byte(entry + 1) & 15
            if group >= self.groups or strap in self.ram_choices:
                raise PostError("invalid or duplicate RAM strap")
            self.ram_choices[strap] = group
        if set(self.ram_choices) != set(range(16)):
            raise PostError("incomplete RAM strap translation")

    def ram_group(self):
        if self.ram_index is not None:
            return self.ram_index
        strap = (self.read(0x101000) & 0x3C) >> 2
        if strap not in self.ram_choices:
            raise PostError("RAM strap has no matching configuration")
        self.ram_index = self.ram_choices[strap]
        return self.ram_index

    def run(self):
        if self.started:
            raise PostError("POST attempt is single-use")
        self.started = True
        self.deadline = self.clock() + self.MAX_SECONDS
        try:
            for root in self.roots:
                self.enabled = True
                self.script(root, 0)
            self.tick()
            self.complete = True
        except Exception as exc:
            self.error = f"pc={self.pc:#x}: {exc}"
            raise
        return {"steps": self.steps, "writes": self.writes, "complete": self.complete}

    def script(self, pc, depth, repeat=False):
        if depth >= self.MAX_DEPTH:
            raise PostError("POST call-depth bound exceeded")
        while True:
            self.pc = pc
            self.tick()
            self.steps += 1
            if self.steps > self.MAX_STEPS:
                raise PostError("POST instruction bound exceeded")
            record = self.records.get(pc)
            if not record:
                raise PostError(f"not a validated instruction: {pc:#x}")
            op = record["opcode"]
            nxt = pc + record["bytes"]
            if op == 0x71:
                if repeat:
                    raise PostError("DONE before END_REPEAT")
                return nxt
            if op == 0x36:
                if not repeat:
                    raise PostError("END_REPEAT without REPEAT")
                return nxt
            if op == 0x33:
                for _ in range(self.byte(pc + 1)):
                    end = self.script(nxt, depth + 1, True)
                nxt = end
            elif op == 0x5B:
                if self.enabled:
                    self.script(self.word(pc + 1), depth + 1)
            elif op == 0x38:
                self.enabled = not self.enabled
            elif op == 0x72:
                self.enabled = True
            elif op == 0x75:
                if not self.condition(self.byte(pc + 1)):
                    self.enabled = False
            elif op == 0x56 and self.enabled:
                matched = False
                for _ in range(min(self.byte(pc + 2) * 50, 100)):
                    if self.condition(self.byte(pc + 1)):
                        matched = True
                        break
                    self.bus.delay_us(20000)
                    self.tick()
                if not matched:
                    self.enabled = False
            elif op == 0x76:
                ptr = self.table(8) + self.byte(pc + 1) * 5
                value = self.bus.vga_read(self.word(ptr), self.byte(ptr + 2)) if self.enabled else 0
                if value & self.byte(ptr + 3) != self.byte(ptr + 4):
                    self.enabled = False
            elif op in (0x8C, 0x8D):
                pass  # Documented reset markers, no hardware operation.
            elif not self.enabled:
                pass
            elif op == 0x74:
                self.bus.delay_us(self.word(pc + 1))
                self.tick()
            elif op == 0x7A:
                addr, value = self.dword(pc + 1), self.dword(pc + 5)
                self.write(addr, value | (1 if addr == 0x200 else 0))
            elif op == 0x6E:
                self.mask(self.dword(pc + 1), self.dword(pc + 5), self.dword(pc + 9))
            elif op in (0x58, 0x91):
                addr = self.dword(pc + 1)
                for i in range(self.byte(pc + 5)):
                    self.write(addr + (4 * i if op == 0x58 else 0),
                               self.dword(pc + 6 + i * 4))
            elif op == 0x8F:
                group = self.ram_group()
                for i in range(self.byte(pc + 6)):
                    self.write(self.dword(pc + 1) + self.byte(pc + 5) * i,
                               self.dword(pc + 7 + 4 * (i * self.groups + group)))
            elif op == 0x6F:
                ptr = self.table(4) + self.byte(pc + 1) * 8
                self.write(self.dword(ptr), self.dword(ptr + 4))
            elif op == 0x90:
                self.write(self.dword(pc + 5), self.read(self.dword(pc + 1)))
            elif op == 0x5F:
                value = (self.shift(self.read(self.dword(pc + 1)), self.byte(pc + 5)) &
                         self.dword(pc + 6)) ^ self.dword(pc + 10)
                self.mask(self.dword(pc + 14), self.dword(pc + 18), value)
            elif op == 0x97:
                addr, mask = self.dword(pc + 1), self.dword(pc + 5)
                old = self.read(addr)
                self.write(addr, (old & mask) | ((old + self.dword(pc + 9)) & ~mask))
            elif op == 0x96:
                index = self.shift(self.read(self.dword(pc + 1)), self.byte(pc + 5)) & self.byte(pc + 6)
                table = self.word(self.table(16) + self.byte(pc + 7) * 2)
                self.mask(self.dword(pc + 8), self.dword(pc + 12),
                          self.byte(table + index) << self.byte(pc + 16))
            elif op in (0x52, 0x53):
                index = self.byte(pc + 1)
                value = self.byte(pc + 2)
                if op == 0x52:
                    value = (self.bus.vga_read(0x3D4, index) & value) | self.byte(pc + 3)
                self.bus.vga_write(0x3D4, index, value)
            elif op == 0x69:
                port = self.word(pc + 1)
                # NV50+ VGA access enable mirrors Nouveau init_io().
                if port == 0x3C3:
                    self.write(0x614100, 0x10000018)
                    self.write(0x614900, 0x10000018)
                self.bus.port_write(port, (self.bus.port_read(port) & self.byte(pc + 3)) |
                                    self.byte(pc + 4))
            elif op == 0x8E:
                self.bus.gpio_reset(self.rom)
            elif op == 0x4C:
                for i in range(self.byte(pc + 3)):
                    ptr = pc + 4 + i * 3
                    bus, address, reg = self.byte(pc + 1), self.byte(pc + 2) >> 1, self.byte(ptr)
                    value = self.bus.i2c_read(bus, address, reg)
                    self.bus.i2c_write(bus, address, reg,
                                       (value & self.byte(ptr + 1)) | self.byte(ptr + 2))
            else:
                raise PostError(f"unimplemented execution at {pc:#x}, opcode={op:#x}")
            pc = nxt
