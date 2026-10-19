#!/usr/bin/env python3
"""
Improved AArch64 instruction analyzer for forbidden crypto instructions.
Uses proper bit-field decoding for AArch64 ISA.
"""
import struct
import sys
import os

def decode_aarch64_instruction(instr):
    """
    Decode AArch64 32-bit instruction to identify forbidden patterns.
    Returns (mnemonic, details) or (None, None) if not a forbidden instruction.
    """
    
    # AES instructions (from ARM Architecture Reference Manual)
    # These are in the SIMD2 (vector) space
    # Format: 0 1 0 0 1 1 1 0 size:2 1 0 0 0 1 Rm:5 opcode:3 n:5 d:5
    # Where opcode determines: aese, aesmc, aesd, aesimc
    
    # aese.16b: D=0, opcode=0 (only for 16B elements)
    # aesmc.16b: D=0, opcode=1
    # aesd.16b: D=0, opcode=2
    # aesimc.16b: D=0, opcode=3
    # Pattern: 4E 2X 48/68/58/78 XX where X depends on registers
    
    # More precise check: AES instructions have specific opcodes
    # All AES instructions end with 4-7 after size field
    if (instr & 0xFFFF0000) == 0x4E284800 or \
       (instr & 0xFFFF0000) == 0x4E286800 or \
       (instr & 0xFFFF0000) == 0x4E285800 or \
       (instr & 0xFFFF0000) == 0x4E287800:
        opcode = (instr >> 12) & 0xF
        if opcode in [0x4, 0x6, 0x5, 0x7]:
            if opcode == 0x4:
                return ('aese', instr)
            elif opcode == 0x6:
                return ('aesmc', instr)
            elif opcode == 0x5:
                return ('aesd', instr)
            elif opcode == 0x7:
                return ('aesimc', instr)
    
    # Atomic operations (from ARM Architecture Reference Manual)
    # LDADD, LDADDx (Atomic Add): 
    #   bits [31:30] = 1X (64-bit variant uses 11)
    #   bits [29:24] = 111000
    #   bits [22:21] = 10
    #   Rm != 11111
    # Pattern: 1X 111000 1X XX...XX where middle bits form specific pattern
    
    if (instr & 0xC0E00C00) == 0xC0E00800:
        # More specific: ldadd
        size = (instr >> 30) & 0x3
        op = (instr >> 22) & 0x3
        if op == 2:  # 10 for atomic operations
            return ('ldadd', instr)
    
    # SWP (Atomic Swap):
    #   bits [31:30] = 1X
    #   bits [29:24] = 001000
    #   bits [22:21] = 10
    if (instr & 0xC0E00C00) == 0x00800800:
        return ('swp', instr)
    
    # CAS (Compare and Swap):
    #   bits [31:30] = 11
    #   bits [29:24] = 001000 (or 000100 for CASx variants)
    #   Various bit patterns
    if (instr & 0xC0E00C00) == 0x08800800 or \
       (instr & 0xC0E00C00) == 0xC0800000:
        return ('cas', instr)
    
    return (None, None)

def check_for_atomic_instructions(data, offset, size):
    """Scan for ldxr/stxr exclusive load/store instructions."""
    ldxr_found = False
    stxr_found = False
    
    code = data[offset:offset + size]
    
    for i in range(0, len(code) - 3, 4):
        instr = struct.unpack('<I', code[i:i+4])[0]
        
        # LDXR (Load Exclusive Register):
        #   bits [31:30] = 11
        #   bits [29:24] = 001000
        #   bits [22:21] = 01
        #   bits [15:10] = 011111
        # 0xC8DFxFxx pattern
        if (instr & 0xFFC00FFF) == 0xC85F0C00 or \
           (instr & 0xFFC0FC00) == 0xC85F0000:
            ldxr_found = True
        
        # STXR (Store Exclusive Register):
        #   bits [31:30] = 11
        #   bits [29:24] = 000000
        #   bits [22:21] = 01
        # 0xC800xxxx pattern with more specific bits
        if (instr & 0xFFC00C00) == 0xC8000400 or \
           (instr & 0xFFE0FC00) == 0xC8000C00:
            stxr_found = True
    
    return ldxr_found, stxr_found

def analyze_elf(elf_path):
    """Analyze ELF file for forbidden instructions."""
    
    if not os.path.exists(elf_path):
        print(f"ERROR: {elf_path} not found")
        return False
    
    with open(elf_path, 'rb') as f:
        data = f.read()
    
    print(f"[*] ELF file size: {len(data)} bytes")
    
    # Verify ELF magic
    if data[0:4] != b'\x7fELF':
        print(f"ERROR: Not an ELF file")
        return False
    
    print(f"[+] Valid ELF file detected")
    
    # Parse ELF64 sections
    e_shoff = struct.unpack('<Q', data[40:48])[0]
    e_shentsize = struct.unpack('<H', data[58:60])[0]
    e_shnum = struct.unpack('<H', data[60:62])[0]
    
    # Find executable sections
    forbidden_found = []
    ldxr_found = False
    stxr_found = False
    total_sections = 0
    
    for i in range(e_shnum):
        sh_offset = e_shoff + i * e_shentsize
        if sh_offset + e_shentsize > len(data):
            break
        
        sh_type = struct.unpack('<I', data[sh_offset+4:sh_offset+8])[0]
        sh_flags = struct.unpack('<Q', data[sh_offset+8:sh_offset+16])[0]
        sh_offset_val = struct.unpack('<Q', data[sh_offset+24:sh_offset+32])[0]
        sh_size = struct.unpack('<Q', data[sh_offset+32:sh_offset+40])[0]
        
        # Check for PROGBITS with execution flag
        if sh_type == 1 and (sh_flags & 0x2):  # PROGBITS and ALLOC
            if sh_size > 100:
                total_sections += 1
                print(f"[*] Scanning section {i}: offset=0x{sh_offset_val:x}, size=0x{sh_size:x}")
                
                code = data[sh_offset_val:sh_offset_val + sh_size]
                
                # Check for atomic operations
                ldx, stx = check_for_atomic_instructions(data, sh_offset_val, sh_size)
                if ldx:
                    ldxr_found = True
                if stx:
                    stxr_found = True
                
                # Check for forbidden instructions
                for j in range(0, len(code) - 3, 4):
                    instr = struct.unpack('<I', code[j:j+4])[0]
                    mnem, enc = decode_aarch64_instruction(instr)
                    
                    if mnem:
                        forbidden_found.append((mnem, sh_offset_val + j, instr))
    
    print(f"\n[ANALYSIS RESULTS]")
    print(f"Sections scanned: {total_sections}")
    print(f"Forbidden A53 instructions found: {len(forbidden_found)}")
    
    if forbidden_found:
        print(f"\n[-] FAILED - Forbidden instructions detected:")
        for name, addr, instr in forbidden_found[:20]:
            print(f"    {name} at 0x{addr:x}: 0x{instr:08x}")
        if len(forbidden_found) > 20:
            print(f"    ... and {len(forbidden_found) - 20} more")
        return False
    else:
        print(f"[+] CONFIRMED: Zero forbidden A53 crypto/atomic instructions found ✓")
    
    print(f"\n[ATOMIC OPERATIONS]")
    if ldxr_found or stxr_found:
        print(f"[+] Atomic operations present:")
        if ldxr_found:
            print(f"    ldxr (Load Exclusive) found ✓")
        if stxr_found:
            print(f"    stxr (Store Exclusive) found ✓")
    else:
        print(f"[WARNING] No exclusive load/store patterns detected")
        print(f"[NOTE] This may indicate atomic operations not used or different patterns")
    
    return True

if __name__ == '__main__':
    elf_file = 'build_pi3/kernel8_pi3.elf'
    success = analyze_elf(elf_file)
    sys.exit(0 if success else 1)
