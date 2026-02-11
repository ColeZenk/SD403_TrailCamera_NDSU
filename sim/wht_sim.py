#!/usr/bin/env python3
"""
SDD501 - Walsh-Hadamard Transform Simulation
Implements both reference (loop) and branchless (hardware) FWHT
for cross-validation.
"""


# =============================================================================
# Fixed-width integer helpers (simulates HDL signal width)
# =============================================================================

BIT_WIDTH = 16  # adjust as needed
W = (1 << BIT_WIDTH) - 1


def bit_not(x):
    """Fixed-width bitwise complement. In HDL this is implicit via signal width."""
    return ~x & W


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
                x[i + j]              = a + b
                x[i + j + stride]     = a - b
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

        # butterfly - two adders in hardware
        a = index.x[index.i + index.j]
        b = index.x[index.i + index.j + index.stride]
        index.x[index.i + index.j]                = a + b
        index.x[index.i + index.j + index.stride]  = a - b

        index.cycle += 1

        # === passive increment logic ===

        # j counter: wraps at stride using AND-NOT mask
        # ((a*!b) + (!a*b)) * a = a * !b
        # by idempotent and complement laws
        index.j = (index.j + 1) & bit_not(index.stride)

        # i counter: advances by stride<<1 when j wraps, wraps at N
        # uses j==0 as carry signal, masks with N-1 to wrap
        j_wrap = int(not index.j)
        index.i = (index.i + (j_wrap * (index.stride << 1))) & (index.N - 1)

        # stride: advances when both j and i have wrapped
        i_wrap = int(not index.i)
        index.stride <<= (j_wrap * i_wrap)

        # done when stride reaches N
        index.done = bool(index.stride & index.N)

    def run(index):
        """Clock until done."""
        while not index.done:
            index.tick()
        return index.x


# =============================================================================
# Matrix form (for triple validation)
# =============================================================================

def hadamard_matrix(N):
    """Build NxN Hadamard matrix recursively."""
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
    """WHT via direct matrix multiply. O(N^2), reference only."""
    N = len(data)
    H = hadamard_matrix(N)
    result = []
    for k in range(N):
        val = 0
        for i in range(N):
            val += H[k][i] * data[i]
        result.append(val)
    return result


# =============================================================================
# Validation
# =============================================================================

def validate(N):
    """Cross-validate all three implementations."""
    import random
    data = [random.randint(-128, 127) for _ in range(N)]

    # three implementations
    ref = fwht_reference(data)
    mat = fwht_matrix(data)

    hw = FWHT(data)
    branchless = hw.run()

    # compare
    assert ref == mat, f"Reference != Matrix for N={N}"
    assert ref == branchless, f"Reference != Branchless for N={N}\n  ref: {ref}\n  hw:  {branchless}"

    # round-trip
    recovered = ifwht_reference(ref)
    assert recovered == data, f"Round-trip failed for N={N}"

    print(f"N={N:4d} | cycles={hw.cycle:5d} | expected={(N >> 1) * N.bit_length() - 1:5d} | PASS")


def main():
    print("Walsh-Hadamard Transform Validation")
    print("=" * 60)

    for exp in range(1, 8):
        N = 1 << exp
        validate(N)

    print("=" * 60)
    print("All tests passed.")

    # demo with small input
    print("\nDemo: N=8")
    data = [1, 2, 3, 4, 5, 6, 7, 8]
    print(f"  Input:   {data}")
    print(f"  WHT:     {fwht_reference(data)}")
    print(f"  Inverse: {ifwht_reference(fwht_reference(data))}")


if __name__ == "__main__":
    main()
