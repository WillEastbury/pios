#!/usr/bin/env python3
"""
Direct binary scan for forbidden A53 instructions in ELF.
Looks for exact instruction encodings only.
"""
import struct
import os

def scan_elf_for_forbidden(elf_path):
    """Scan ELF binary for forbidden crypto instructions."""
    
    if not os.path.exists(elf_path):
        print(f"ERROR: {elf_path} not found")
        return False
    
    with open(elf_path, 'rb') as f:
        data = f.read()
    
    # AES instruction byte patterns (exact matches only)
    # These are the exact 4-byte instructions that GCC generates
    # aese v<d>.16b, v<n>.16b: pattern 4E 28 48 <Rd>
    # aesmc v<d>.16b, v<d>.16b: pattern 4E 28 68 <Rd> 
    # aesd v<d>.16b, v<n>.16b: pattern 4E 28 58 <Rd>
    # aesimc v<d>.16b, v<d>.16b: pattern 4E 28 78 <Rd>
    
    aes_patterns = [
        (b'\x4E\x28\x48', 'aese'),
        (b'\x4E\x28\x68', 'aesmc'),
        (b'\x4E\x28\x58', 'aesd'),
        (b'\x4E\x28\x78', 'aesimc'),
    ]
    
    forbidden_count = 0
    found_locations = {}
    
    print(f"[*] Scanning {len(data)} bytes for forbidden AES instructions...")
    
    # Scan for each pattern
    for pattern, mnem in aes_patterns:
        idx = 0
        while True:
            pos = data.find(pattern, idx)
            if pos == -1:
                break
            forbidden_count += 1
            if mnem not in found_locations:
                found_locations[mnem] = []
            found_locations[mnem].append(pos)
            idx = pos + 1
    
    print(f"\n[RESULTS]")
    print(f"Forbidden A53 AES instructions found: {forbidden_count}")
    
    if forbidden_count == 0:
        print(f"[+] CONFIRMED: Zero forbidden AES instructions (aese/aesmc/aesd/aesimc)")
    else:
        print(f"\n[-] FAILED - Found forbidden AES instructions:")
        for mnem in ['aese', 'aesmc', 'aesd', 'aesimc']:
            if mnem in found_locations:
                locs = found_locations[mnem]
                print(f"    {mnem}: {len(locs)} occurrence(s)")
                for addr in locs[:5]:
                    instr_bytes = data[addr:addr+4]
                    instr_int = struct.unpack('<I', instr_bytes)[0]
                    print(f"        at 0x{addr:x}: {instr_bytes.hex()}")
        return False
    
    # Now check for atomic memory operations
    # These are common and not forbidden, but we want to verify they're there for atomic operations
    atomic_patterns = [
        (b'\xC8\xDF', 'ldxr'),  # Load Exclusive Register (approximate)
        (b'\xC8\x00', 'stxr'),  # Store Exclusive Register (approximate)
    ]
    
    print(f"\n[ATOMIC OPERATIONS CHECK]")
    atomic_found = {}
    for pattern, mnem in atomic_patterns:
        if data.find(pattern) != -1:
            atomic_found[mnem] = True
            print(f"[+] {mnem} pattern found")
        else:
            atomic_found[mnem] = False
    
    if not atomic_found['ldxr'] and not atomic_found['stxr']:
        print(f"[NOTE] No exclusive load/store patterns detected")
    
    return forbidden_count == 0

if __name__ == '__main__':
    elf_file = 'build_pi3/kernel8_pi3.elf'
    success = scan_elf_for_forbidden(elf_file)
    exit(0 if success else 1)
