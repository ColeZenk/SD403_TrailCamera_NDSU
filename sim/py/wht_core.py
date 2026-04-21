"""
WHT core — 1D/2D Walsh-Hadamard transforms.

Contains both the reference (loop) implementation and the branchless
state machine that maps directly to the FPGA butterfly in wht8.v.
"""

import random

# =============================================================================
# Fixed-width integer helpers (simulates HDL signal width)
# =============================================================================

BIT_WIDTH = 18                       # W=18 matches FPGA (SDD401)
W = (1 << BIT_WIDTH) - 1
SMAX =  (1 << (BIT_WIDTH - 1)) - 1  # +131071
SMIN = -(1 << (BIT_WIDTH - 1))      # -131072


def bit_not(x):
    """Fixed-width bitwise complement. In HDL this is implicit via signal width."""
    return ~x & W


def clamp(x):
    """Signed clamp to W-bit range. Models overflow behavior of HDL signals."""
    if x > SMAX:
        return SMAX
    if x < SMIN:
        return SMIN
    return x


# =============================================================================
# Reference implementation (loop-based, for validation)
# =============================================================================

def fwht_reference(data):
    """Standard FWHT with explicit loops. O(N log N)."""
    x = data[:]
    N = len(x)
    stride = 1
    while stride < N:
        for i in range(0, N, stride << 1):
            for j in range(stride):
                a = x[i + j]
                b = x[i + j + stride]
                x[i + j]          = a + b
                x[i + j + stride] = a - b
        stride <<= 1
    return x


def ifwht_reference(data):
    """Inverse FWHT. Same transform, scaled by 1/N."""
    x = fwht_reference(data)
    N = len(x)
    return [v // N for v in x]


# =============================================================================
# Branchless state machine (maps directly to HDL)
# =============================================================================

class FWHT:
    def __init__(index, data):
        index.x = data[:]
        index.N = len(data)
        index.stride = 1
        index.i = 0
        index.j = 0
        index.done = False
        index.cycle = 0

    def tick(index):
        if index.done:
            return

        a = index.x[index.i + index.j]
        b = index.x[index.i + index.j + index.stride]
        index.x[index.i + index.j]               = clamp(a + b)
        index.x[index.i + index.j + index.stride] = clamp(a - b)

        index.cycle += 1

        index.j = (index.j + 1) & bit_not(index.stride)

        j_wrap = int(not index.j)
        index.i = (index.i + (j_wrap * (index.stride << 1))) & (index.N - 1)

        i_wrap = int(not index.i)
        index.stride <<= (j_wrap * i_wrap)

        index.done = bool(index.stride & index.N)

    def run(index):
        while not index.done:
            index.tick()
        return index.x


# =============================================================================
# 2D transforms (8x8 block)
# =============================================================================

def wht_2D(block):
    result = []
    for row in block:
        result.append(FWHT(list(row)).run())
    for c in range(8):
        col = [result[r][c] for r in range(8)]
        col = FWHT(col).run()
        for r in range(8):
            result[r][c] = col[r]
    return result


def iwht_2D(coeffs):
    """Inverse 2D WHT — same butterfly, divide by 64."""
    result = [list(row) for row in coeffs]
    for r in range(8):
        result[r] = fwht_reference(result[r])
    for c in range(8):
        col = fwht_reference([result[r][c] for r in range(8)])
        for r in range(8):
            result[r][c] = col[r]
    return [[v / 64.0 for v in row] for row in result]


# =============================================================================
# Matrix form (for triple validation)
# =============================================================================

def hadamard_matrix(N):
    if N == 1:
        return [[1]]
    half = hadamard_matrix(N >> 1)
    H = []
    for row in half:
        H.append(row + row)
    for row in half:
        H.append(row + [-x for x in row])
    return H


def fwht_matrix(data):
    N = len(data)
    H = hadamard_matrix(N)
    return [sum(H[k][i] * data[i] for i in range(N)) for k in range(N)]


# =============================================================================
# Validation
# =============================================================================

def validate(N):
    data = [random.randint(-128, 127) for _ in range(N)]

    ref        = fwht_reference(data)
    mat        = fwht_matrix(data)
    hw         = FWHT(data)
    branchless = hw.run()

    assert ref == mat,        f"Reference != Matrix for N={N}"
    assert ref == branchless, f"Reference != Branchless for N={N}"

    recovered = ifwht_reference(ref)
    assert recovered == data, f"Round-trip failed for N={N}"

    expected = (N >> 1) * N.bit_length() - 1
    return {'N': N, 'cycles': hw.cycle, 'expected': expected, 'passed': True}
