"""picobrotli -- minimal RFC 7932 (Brotli) encoder + decoder.

Byte-identical Python port of vm/picobrotli.c (vendored from the picoweb codec).
Produces valid Brotli streams decodable by any browser / zlib / Node, using
LZ77 + canonical Huffman, a single meta-block (WBITS=16, no static dictionary,
no context modeling), with an uncompressed meta-block fallback. The decoder
reads the subset this encoder emits (plus uncompressed meta-blocks).

Kept byte-for-byte in lockstep with vm/picobrotli.c and vm/picobrotli.js so
Compress.Brotli* is identical on the Python, JS and C VMs. See the C file for
the authoritative comments on each step.
"""

from __future__ import annotations

MASK32 = 0xFFFFFFFF
MAX_HUFF_BITS = 15

# LZ77 parameters (mirror picobrotli.c)
HASH_BITS = 15
HASH_SIZE = 1 << HASH_BITS
WIN_SIZE = 1 << 16
MIN_MATCH = 4
MAX_MATCH = 258
MAX_CHAIN = 32

# RFC 7932 code-length code order
kCLOrder = (1, 2, 3, 4, 0, 5, 17, 6, 16, 7, 8, 9, 10, 11, 12, 13, 14, 15)
# Fixed prefix code for code_length_code_lengths values 0-5
kCLCL_val = (0, 7, 3, 2, 1, 15)
kCLCL_len = (2, 4, 3, 2, 2, 4)

# Insert / copy length code tables (base, extra-bits)
kInsLen = (
    (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 1), (8, 1),
    (10, 2), (14, 2), (18, 3), (26, 3), (34, 4), (50, 4), (66, 5), (98, 5),
    (130, 6), (194, 7), (322, 8), (578, 9), (1090, 10), (2114, 12),
    (6210, 14), (22594, 24),
)
kCopyLen = (
    (2, 0), (3, 0), (4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0),
    (10, 1), (12, 1), (14, 2), (18, 2), (22, 3), (30, 3), (38, 4), (54, 4),
    (70, 5), (102, 5), (134, 6), (198, 7), (326, 8), (582, 9),
    (1094, 10), (2118, 24),
)


# ================================================================
# Bit writer (LSB-first). Faithful to the byte stream picobrotli.c emits.
# ================================================================
class _BitW:
    __slots__ = ("out", "accum", "nbits")

    def __init__(self) -> None:
        self.out = bytearray()
        self.accum = 0
        self.nbits = 0

    def put(self, val: int, n: int) -> None:
        self.accum |= (val << self.nbits)
        self.nbits += n
        while self.nbits >= 8:
            self.out.append(self.accum & 0xFF)
            self.accum >>= 8
            self.nbits -= 8

    def align(self) -> None:
        # Pad to a byte boundary (encode_stored): only when not already aligned.
        if self.nbits > 0:
            pad = 8 - (self.nbits % 8)
            if pad < 8:
                self.put(0, pad)

    def finish(self) -> None:
        if self.nbits > 0:
            self.out.append(self.accum & 0xFF)
            self.accum = 0
            self.nbits = 0


def _count_used(freq, n):
    c = 0
    for i in range(n):
        if freq[i]:
            c += 1
    return c


def _build_lengths_ex(freq, nsym, max_bits):
    """Two-queue Huffman + Kraft fixup. Returns a list `lens` of length nsym."""
    lens = [0] * nsym

    sorted_ = [i for i in range(nsym) if freq[i]]
    nused = len(sorted_)
    if nused == 0:
        return lens
    if nused == 1:
        lens[sorted_[0]] = 1
        return lens

    # Insertion sort by frequency ascending (stable for equal freqs)
    for i in range(1, nused):
        key = sorted_[i]
        kf = freq[key]
        j = i - 1
        while j >= 0 and freq[sorted_[j]] > kf:
            sorted_[j + 1] = sorted_[j]
            j -= 1
        sorted_[j + 1] = key

    # Two-queue merge to build the tree
    cap = 2 * nused
    nf = [0] * cap
    par = [-1] * cap
    for i in range(nused):
        nf[i] = freq[sorted_[i]]
        par[i] = -1

    nn = nused
    q1 = 0
    q2buf = [0] * cap
    q2h = 0
    q2t = 0

    for _m in range(nused - 1):
        pick = [0, 0]
        for p in range(2):
            h1 = q1 < nused
            h2 = q2h < q2t
            if h1 and h2:
                if nf[q1] <= nf[q2buf[q2h]]:
                    pick[p] = q1
                    q1 += 1
                else:
                    pick[p] = q2buf[q2h]
                    q2h += 1
            elif h1:
                pick[p] = q1
                q1 += 1
            else:
                pick[p] = q2buf[q2h]
                q2h += 1
        nf[nn] = nf[pick[0]] + nf[pick[1]]
        par[nn] = -1
        par[pick[0]] = nn
        par[pick[1]] = nn
        q2buf[q2t] = nn
        q2t += 1
        nn += 1

    # Depth of each leaf, clamped to max_bits
    for i in range(nused):
        d = 0
        cur = i
        while par[cur] != -1:
            cur = par[cur]
            d += 1
        if d > max_bits:
            d = max_bits
        lens[sorted_[i]] = d

    # Kraft inequality adjustment
    for _it in range(50):
        kraft = 0
        for i in range(nsym):
            if lens[i]:
                kraft += (1 << (max_bits - lens[i]))
        target = (1 << max_bits)
        if kraft == target:
            break
        if kraft > target:
            l = 1
            while l < max_bits and kraft > target:
                i = 0
                while i < nsym and kraft > target:
                    if lens[i] == l:
                        lens[i] += 1
                        kraft -= (1 << (max_bits - l))
                        kraft += (1 << (max_bits - l - 1))
                    i += 1
                l += 1
        else:
            l = max_bits
            while l > 1 and kraft < target:
                i = nsym - 1
                while i >= 0 and kraft < target:
                    if lens[i] == l:
                        lens[i] -= 1
                        kraft -= (1 << (max_bits - l))
                        kraft += (1 << (max_bits - l + 1))
                    i -= 1
                l -= 1

    return lens


def _build_lengths(freq, nsym):
    return _build_lengths_ex(freq, nsym, MAX_HUFF_BITS)


def _assign_codes(lens, nsym):
    """Returns list of [code, length]."""
    bl_count = [0] * (MAX_HUFF_BITS + 1)
    for i in range(nsym):
        if lens[i]:
            bl_count[lens[i]] += 1
    nxt = [0] * (MAX_HUFF_BITS + 1)
    c = 0
    for b in range(1, MAX_HUFF_BITS + 1):
        c = (c + bl_count[b - 1]) << 1
        nxt[b] = c & 0xFFFF
    codes = [[0, 0] for _ in range(nsym)]
    for i in range(nsym):
        codes[i][1] = lens[i]
        if lens[i]:
            codes[i][0] = nxt[lens[i]]
            nxt[lens[i]] = (nxt[lens[i]] + 1) & 0xFFFF
        else:
            codes[i][0] = 0
    return codes


def _bw_huff(w: _BitW, code) -> None:
    c_code, c_len = code[0], code[1]
    rev = 0
    for i in range(c_len):
        rev |= (((c_code >> i) & 1) << (c_len - 1 - i))
    w.put(rev, c_len)


# ================================================================
# Prefix code transmission
# ================================================================
def _write_simple_code(w: _BitW, lens, nsym, alpha_bits) -> None:
    used = []
    for i in range(nsym):
        if lens[i] and len(used) < 4:
            used.append(i)
    if len(used) == 0:
        used = [0]
    used.sort()
    nu = len(used)
    w.put(1, 2)            # type = simple (HSKIP=1)
    w.put(nu - 1, 2)       # NSYM - 1
    for i in range(nu):
        w.put(used[i], alpha_bits)
    if nu == 4:
        w.put(1 if lens[used[0]] == 1 else 0, 1)


def _write_complex_code(w: _BitW, lens, nsym) -> None:
    cl_syms = []
    cl_extra = []

    last_nz = nsym - 1
    while last_nz > 0 and lens[last_nz] == 0:
        last_nz -= 1
    cl_end = last_nz + 1

    i = 0
    while i < cl_end:
        if lens[i] == 0:
            run = 0
            while i + run < cl_end and lens[i + run] == 0:
                run += 1
            prev_was_17 = False
            while run > 0:
                if run >= 3 and not prev_was_17:
                    r = 10 if run > 10 else run
                    cl_syms.append(17)
                    cl_extra.append(r - 3)
                    run -= r
                    i += r
                    prev_was_17 = True
                else:
                    cl_syms.append(0)
                    cl_extra.append(0)
                    run -= 1
                    i += 1
                    prev_was_17 = False
        else:
            cl_syms.append(lens[i])
            cl_extra.append(0)
            i += 1

    cl_n = len(cl_syms)

    cl_freq = [0] * 18
    for s in cl_syms:
        cl_freq[s] += 1

    cl_used = 0
    for k in range(18):
        if cl_freq[k]:
            cl_used += 1
    if cl_used == 1:
        if cl_freq[0] == 0:
            dummy = 0
        elif cl_freq[1] == 0:
            dummy = 1
        else:
            dummy = 2
        cl_freq[dummy] = 1

    cl_lens = _build_lengths_ex(cl_freq, 18, 5)
    cl_codes = _assign_codes(cl_lens, 18)

    num_cl = 18
    while num_cl > 4 and cl_lens[kCLOrder[num_cl - 1]] == 0:
        num_cl -= 1

    hskip = 0
    if num_cl > 3 and cl_lens[kCLOrder[0]] == 0 and cl_lens[kCLOrder[1]] == 0:
        if cl_lens[kCLOrder[2]] == 0:
            hskip = 3
        else:
            hskip = 2
    if hskip == 1:
        hskip = 0

    w.put(hskip, 2)

    for k in range(hskip, num_cl):
        v = cl_lens[kCLOrder[k]]
        w.put(kCLCL_val[v], kCLCL_len[v])

    for k in range(cl_n):
        _bw_huff(w, cl_codes[cl_syms[k]])
        if cl_syms[k] == 17:
            w.put(cl_extra[k], 3)


def _write_prefix(w: _BitW, freq, lens, nsym, alpha_bits, codes) -> None:
    """Writes the prefix code and rewrites lens/codes to match a simple code."""
    nu = _count_used(freq, nsym)
    if nu <= 4:
        _write_simple_code(w, lens, nsym, alpha_bits)
        used = []
        for i in range(nsym):
            if freq[i] and len(used) < 4:
                used.append(i)
        used.sort()
        n = len(used)

        for i in range(nsym):
            codes[i][0] = 0
            codes[i][1] = 0
            lens[i] = 0

        if n == 1:
            lens[used[0]] = 0
            codes[used[0]][0] = 0
            codes[used[0]][1] = 0
        elif n == 2:
            lens[used[0]] = 1
            codes[used[0]][0] = 0
            codes[used[0]][1] = 1
            lens[used[1]] = 1
            codes[used[1]][0] = 1
            codes[used[1]][1] = 1
        elif n == 3:
            lens[used[0]] = 1
            codes[used[0]][0] = 0
            codes[used[0]][1] = 1
            lens[used[1]] = 2
            codes[used[1]][0] = 2
            codes[used[1]][1] = 2
            lens[used[2]] = 2
            codes[used[2]][0] = 3
            codes[used[2]][1] = 2
        elif n == 4:
            tree_sel = (lens[used[0]] == 1)
            if tree_sel:
                lens[used[0]] = 1
                codes[used[0]][0] = 0
                codes[used[0]][1] = 1
                lens[used[1]] = 2
                codes[used[1]][0] = 2
                codes[used[1]][1] = 2
                lens[used[2]] = 3
                codes[used[2]][0] = 6
                codes[used[2]][1] = 3
                lens[used[3]] = 3
                codes[used[3]][0] = 7
                codes[used[3]][1] = 3
            else:
                for idx, cd in ((0, 0), (1, 1), (2, 2), (3, 3)):
                    lens[used[idx]] = 2
                    codes[used[idx]][0] = cd
                    codes[used[idx]][1] = 2
    else:
        _write_complex_code(w, lens, nsym)


# ================================================================
# LZ77 match finder
# ================================================================
def _hash4(b, p):
    v = b[p] | (b[p + 1] << 8) | (b[p + 2] << 16) | (b[p + 3] << 24)
    return ((v * 0x1E35A7BD) & MASK32) >> (32 - HASH_BITS)


def _lz_parse(data):
    n = len(data)
    if n == 0:
        return []
    head = [-1] * HASH_SIZE
    prev = [0] * n
    cmds = []  # each: [ins_len, copy_len, distance]

    ip = 0
    lit_start = 0
    while ip < n:
        best_len = 0
        best_dist = 0
        if ip + MIN_MATCH <= n:
            h = _hash4(data, ip)
            chain = head[h]
            cc = 0
            while chain >= 0 and cc < MAX_CHAIN:
                dist = ip - chain
                if dist > WIN_SIZE:
                    break
                maxl = n - ip
                if maxl > MAX_MATCH:
                    maxl = MAX_MATCH
                ml = 0
                while ml < maxl and data[chain + ml] == data[ip + ml]:
                    ml += 1
                if ml > best_len and ml >= MIN_MATCH:
                    best_len = ml
                    best_dist = dist
                    if best_len >= MAX_MATCH:
                        break
                chain = prev[chain]
                cc += 1
            prev[ip] = head[h]
            head[h] = ip

        if best_len >= MIN_MATCH:
            cmds.append([ip - lit_start, best_len, best_dist])
            k = 1
            while k < best_len and ip + k + MIN_MATCH <= n:
                hk = _hash4(data, ip + k)
                prev[ip + k] = head[hk]
                head[hk] = ip + k
                k += 1
            ip += best_len
            lit_start = ip
        else:
            ip += 1

    if lit_start < n:
        cmds.append([n - lit_start, 0, 0])
    return cmds


def _find_ins_code(v):
    for i in range(23, -1, -1):
        if v >= kInsLen[i][0]:
            return i, v - kInsLen[i][0], kInsLen[i][1]
    return 0, 0, 0


def _find_copy_code(v):
    for i in range(23, -1, -1):
        if v >= kCopyLen[i][0]:
            return i, v - kCopyLen[i][0], kCopyLen[i][1]
    return 0, 0, 0


def _ic_symbol(ic, cc, use_dist):
    ic_off = ic % 8
    cc_off = cc % 8
    val = ic_off * 8 + cc_off
    if not use_dist:
        if cc < 8:
            return 0 + val
        return 64 + val
    if ic < 8:
        if cc < 8:
            return 128 + val
        if cc < 16:
            return 192 + val
        return 384 + val
    if ic < 16:
        if cc < 8:
            return 256 + val
        if cc < 16:
            return 320 + val
        return 512 + val
    if cc < 8:
        return 448 + val
    if cc < 16:
        return 576 + val
    return 640 + val


def _find_dist_code(dist):
    if dist == 0:
        return 0, 0, 0
    d = dist - 1
    for hcode in range(48):
        nb = 1 + (hcode >> 1)
        off = ((2 + (hcode & 1)) << nb) - 4
        if d >= off and d - off < (1 << nb):
            return 16 + hcode, d - off, nb
    return 16, 0, 0


# ================================================================
# Uncompressed meta-block (fallback)
# ================================================================
def _encode_stored(data):
    n = len(data)
    if n > 0xFFFFFF:
        raise ValueError("too large for stored")
    w = _BitW()
    w.put(0, 1)  # WBITS=16
    remaining = n
    ptr = 0
    while remaining > 0:
        chunk = remaining
        if chunk > (1 << 24) - 1:
            chunk = (1 << 24) - 1
        w.put(0, 1)  # ISLAST=0
        mlen = chunk - 1
        mn = 4 if mlen < (1 << 16) else (5 if mlen < (1 << 20) else 6)
        w.put(mn - 4, 2)
        w.put(mlen, mn * 4)
        w.put(1, 1)  # ISUNCOMPRESSED=1
        w.align()
        w.finish()
        w.out += data[ptr:ptr + chunk]
        ptr += chunk
        remaining -= chunk
    # Final empty meta-block
    w.put(1, 1)  # ISLAST
    w.put(1, 1)  # ISLASTEMPTY
    w.finish()
    return bytes(w.out)


# ================================================================
# Main encoder
# ================================================================
def encode(data) -> bytes:
    data = bytes(data)
    n = len(data)
    if n == 0:
        return b"\x06"  # WBITS=16 + empty last meta-block
    if n > 16 * 1024 * 1024:
        raise ValueError("input too large")

    cmds = _lz_parse(data)

    lit_freq = [0] * 256
    ic_freq = [0] * 704
    dist_freq = [0] * 64

    lp = 0
    for ins_len, copy_len, distance in cmds:
        icode, _ie, _ieb = _find_ins_code(ins_len)
        ccode = 0
        if copy_len:
            ccode, _ce, _ceb = _find_copy_code(copy_len)
        has_dist = copy_len > 0
        sym = _ic_symbol(icode, ccode, has_dist)
        if 0 <= sym < 704:
            ic_freq[sym] += 1
        j = 0
        while j < ins_len and lp < n:
            lit_freq[data[lp]] += 1
            lp += 1
            j += 1
        if copy_len:
            dc, _de, _deb = _find_dist_code(distance)
            if dc < 64:
                dist_freq[dc] += 1
            lp += copy_len

    lit_lens = _build_lengths(lit_freq, 256)
    ic_lens = _build_lengths(ic_freq, 704)
    dist_lens = _build_lengths(dist_freq, 64)

    lit_codes = _assign_codes(lit_lens, 256)
    ic_codes = _assign_codes(ic_lens, 704)
    dist_codes = _assign_codes(dist_lens, 64)

    w = _BitW()
    w.put(0, 1)        # WBITS=16
    w.put(1, 1)        # ISLAST
    w.put(0, 1)        # ISLASTEMPTY=0

    mlen = n - 1
    mn = 4 if mlen < (1 << 16) else (5 if mlen < (1 << 20) else 6)
    w.put(mn - 4, 2)
    w.put(mlen, mn * 4)

    w.put(0, 1)        # NBLTYPESL=1
    w.put(0, 1)        # NBLTYPESI=1
    w.put(0, 1)        # NBLTYPESD=1
    w.put(0, 2)        # NPOSTFIX=0
    w.put(0, 4)        # NDIRECT=0
    w.put(0, 2)        # context mode
    w.put(0, 1)        # NTREESL=1
    w.put(0, 1)        # NTREESD=1

    _write_prefix(w, lit_freq, lit_lens, 256, 8, lit_codes)
    _write_prefix(w, ic_freq, ic_lens, 704, 10, ic_codes)
    _write_prefix(w, dist_freq, dist_lens, 64, 6, dist_codes)

    lp = 0
    for ins_len, copy_len, distance in cmds:
        icode, ie, ieb = _find_ins_code(ins_len)
        ccode = 0
        ce = 0
        ceb = 0
        if copy_len:
            ccode, ce, ceb = _find_copy_code(copy_len)
        has_dist = copy_len > 0
        sym = _ic_symbol(icode, ccode, has_dist)

        _bw_huff(w, ic_codes[sym])
        if ieb > 0:
            w.put(ie, ieb)
        if has_dist and ceb > 0:
            w.put(ce, ceb)

        j = 0
        while j < ins_len and lp < n:
            _bw_huff(w, lit_codes[data[lp]])
            lp += 1
            j += 1

        if has_dist:
            dc, de, deb = _find_dist_code(distance)
            _bw_huff(w, dist_codes[dc])
            if deb > 0:
                w.put(de, deb)
            lp += copy_len

    w.finish()

    if len(w.out) >= n:
        return _encode_stored(data)
    return bytes(w.out)


def bound(input_len: int) -> int:
    return input_len + input_len // 64 + 64


# ================================================================
# Decoder
# ================================================================
class _BitR:
    __slots__ = ("p", "len", "bit")

    def __init__(self, data) -> None:
        self.p = data
        self.len = len(data)
        self.bit = 0

    def read(self, n):
        if n < 0 or n > 24 or self.bit + n > self.len * 8:
            return None
        v = 0
        for i in range(n):
            bi = self.bit
            self.bit += 1
            v |= (((self.p[bi >> 3] >> (bi & 7)) & 1) << i)
        return v

    def align_byte(self):
        self.bit = (self.bit + 7) & ~7


def _bit_reverse(v, n):
    r = 0
    for _i in range(n):
        r = ((r << 1) | (v & 1)) & 0xFFFF
        v >>= 1
    return r


class _HDec:
    __slots__ = ("length", "code", "nsym", "single_symbol")

    def __init__(self):
        self.length = None
        self.code = None
        self.nsym = 0
        self.single_symbol = -1


def _hdec_build(lens, nsym):
    h = _HDec()
    h.length = [0] * nsym
    h.code = [0] * nsym
    h.single_symbol = -1
    h.nsym = nsym
    bl_count = [0] * 16
    for i in range(nsym):
        if lens[i] > 15:
            return None
        h.length[i] = lens[i]
        if lens[i]:
            bl_count[lens[i]] += 1
    nxt = [0] * 16
    c = 0
    for b in range(1, 16):
        c = ((c + bl_count[b - 1]) << 1) & 0xFFFF
        nxt[b] = c
    for i in range(nsym):
        if h.length[i]:
            canon = nxt[h.length[i]]
            nxt[h.length[i]] = (nxt[h.length[i]] + 1) & 0xFFFF
            h.code[i] = _bit_reverse(canon, h.length[i])
    return h


def _hdec_symbol(r, h):
    if h.single_symbol >= 0:
        return h.single_symbol
    code = 0
    for length in range(1, 16):
        bit = r.read(1)
        if bit is None:
            return -1
        code |= (bit << (length - 1))
        for s in range(h.nsym):
            if h.length[s] == length and h.code[s] == code:
                return s
    return -1


def _read_clcl_symbol(r):
    code = 0
    for length in range(1, 5):
        bit = r.read(1)
        if bit is None:
            return -1
        code |= (bit << (length - 1))
        for v in range(6):
            if kCLCL_len[v] == length and kCLCL_val[v] == code:
                return v
    return -1


def _read_prefix_code(r, nsym, alpha_bits):
    lens = [0] * nsym
    hskip = r.read(2)
    if hskip is None:
        return None

    if hskip == 1:
        nsym_m1 = r.read(2)
        if nsym_m1 is None:
            return None
        nn = nsym_m1 + 1
        used = [0, 0, 0, 0]
        for i in range(nn):
            sym = r.read(alpha_bits)
            if sym is None or sym >= nsym:
                return None
            used[i] = sym
        if nn == 1:
            h = _hdec_build(lens, nsym)
            if h is None:
                return None
            h.single_symbol = used[0]
            return h
        elif nn == 2:
            lens[used[0]] = 1
            lens[used[1]] = 1
        elif nn == 3:
            lens[used[0]] = 1
            lens[used[1]] = 2
            lens[used[2]] = 2
        else:
            tree_sel = r.read(1)
            if tree_sel is None:
                return None
            if tree_sel:
                lens[used[0]] = 1
                lens[used[1]] = 2
                lens[used[2]] = 3
                lens[used[3]] = 3
            else:
                lens[used[0]] = 2
                lens[used[1]] = 2
                lens[used[2]] = 2
                lens[used[3]] = 2
        return _hdec_build(lens, nsym)

    if hskip > 3:
        return None
    cl_lens = [0] * 18
    space = 0
    i = hskip
    while i < 18:
        v = _read_clcl_symbol(r)
        if v < 0:
            return None
        cl_lens[kCLOrder[i]] = v
        if v:
            space += 1 << (5 - v)
        if i + 1 >= 4 and space == 32:
            break
        if space > 32:
            return None
        i += 1

    clh = _hdec_build(cl_lens, 18)
    if clh is None:
        return None

    pos = 0
    code_space = 0
    while pos < nsym and code_space < (1 << 15):
        sym = _hdec_symbol(r, clh)
        if sym < 0:
            return None
        if sym == 17:
            extra = r.read(3)
            if extra is None:
                return None
            run = 3 + extra
            if pos + run > nsym:
                return None
            pos += run
        elif 0 <= sym <= 15:
            lens[pos] = sym
            pos += 1
            if sym:
                code_space += 1 << (15 - sym)
                if code_space > (1 << 15):
                    return None
        else:
            return None
    if code_space != (1 << 15):
        return None
    return _hdec_build(lens, nsym)


def _decode_ic_symbol(sym):
    """Returns (ic, cc, explicit_dist) or None."""
    if sym < 0 or sym >= 704:
        return None
    if sym < 64:
        base = sym
        return base >> 3, base & 7, False
    if sym < 128:
        base = sym - 64
        return base >> 3, 8 + (base & 7), False
    if sym < 192:
        base = sym - 128
        return base >> 3, base & 7, True
    if sym < 256:
        base = sym - 192
        return base >> 3, 8 + (base & 7), True
    if sym < 320:
        base = sym - 256
        return 8 + (base >> 3), base & 7, True
    if sym < 384:
        base = sym - 320
        return 8 + (base >> 3), 8 + (base & 7), True
    if sym < 448:
        base = sym - 384
        return base >> 3, 16 + (base & 7), True
    if sym < 512:
        base = sym - 448
        return 16 + (base >> 3), base & 7, True
    if sym < 576:
        base = sym - 512
        return 8 + (base >> 3), 16 + (base & 7), True
    if sym < 640:
        base = sym - 576
        return 16 + (base >> 3), 8 + (base & 7), True
    base = sym - 640
    return 16 + (base >> 3), 16 + (base & 7), True


def _decode_distance(r, dc):
    if dc < 16 or dc >= 64:
        return None
    hcode = dc - 16
    nb = 1 + (hcode >> 1)
    extra = r.read(nb)
    if extra is None:
        return None
    off = ((2 + (hcode & 1)) << nb) - 4
    return off + extra + 1


def decode(data) -> bytes:
    data = bytes(data)
    r = _BitR(data)
    out = bytearray()

    v = r.read(1)
    if v is None or v != 0:
        raise ValueError("bad WBITS")

    while True:
        islast = r.read(1)
        if islast is None:
            raise ValueError("truncated")
        if islast:
            islastempty = r.read(1)
            if islastempty is None:
                raise ValueError("truncated")
            if islastempty:
                return bytes(out)
        mn = r.read(2)
        if mn is None or mn > 2:
            raise ValueError("bad MNIBBLES")
        nibbles = 4 + mn
        mlen_m1 = r.read(nibbles * 4)
        if mlen_m1 is None:
            raise ValueError("truncated")
        mlen = mlen_m1 + 1

        if not islast:
            isuncompressed = r.read(1)
            if isuncompressed is None:
                raise ValueError("truncated")
            if isuncompressed:
                r.align_byte()
                start = r.bit >> 3
                if start + mlen > r.len:
                    raise ValueError("truncated stored")
                out += data[start:start + mlen]
                r.bit += mlen * 8
                continue

        _decode_compressed_meta(r, mlen, out)
        if islast:
            return bytes(out)


def _decode_compressed_meta(r, mlen, out):
    for _k in range(8):
        v = r.read([1, 1, 1, 2, 4, 2, 1, 1][_k])
        if v is None or v != 0:
            raise ValueError("unsupported meta-block")
    lit_h = _read_prefix_code(r, 256, 8)
    if lit_h is None:
        raise ValueError("bad literal code")
    ic_h = _read_prefix_code(r, 704, 10)
    if ic_h is None:
        raise ValueError("bad ic code")
    dist_h = _read_prefix_code(r, 64, 6)
    if dist_h is None:
        raise ValueError("bad dist code")

    end = len(out) + mlen
    while len(out) < end:
        sym = _hdec_symbol(r, ic_h)
        dec = _decode_ic_symbol(sym)
        if dec is None:
            raise ValueError("bad ic symbol")
        ic, cc, explicit_dist = dec
        extra = r.read(kInsLen[ic][1])
        if extra is None:
            raise ValueError("truncated ins")
        ins_len = kInsLen[ic][0] + extra
        copy_len = 0
        if explicit_dist:
            extra = r.read(kCopyLen[cc][1])
            if extra is None:
                raise ValueError("truncated copy")
            copy_len = kCopyLen[cc][0] + extra
        if len(out) + ins_len > end:
            raise ValueError("ins overflow")
        for _i in range(ins_len):
            lit = _hdec_symbol(r, lit_h)
            if lit < 0 or lit > 255:
                raise ValueError("bad literal")
            out.append(lit)
        if explicit_dist:
            dc = _hdec_symbol(r, dist_h)
            dist = _decode_distance(r, dc)
            if dist is None:
                raise ValueError("bad distance")
            if dist == 0 or dist > len(out) or len(out) + copy_len > end + (mlen):
                # match-copy bounds (mirror copy_match: dist<=pos, pos+len<=cap)
                pass
            if dist == 0 or dist > len(out):
                raise ValueError("bad back-distance")
            src = len(out) - dist
            for i in range(copy_len):
                out.append(out[src + i])
            if len(out) > end:
                raise ValueError("copy overflow")


# Public buffer API aliases (parallel to picocompress: compress/decompress)
def compress(data) -> bytes:
    return encode(data)


def decompress(data) -> bytes:
    return decode(data)
