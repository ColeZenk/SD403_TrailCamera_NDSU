#!/usr/bin/env python3
"""
SDD501 - Walsh-Hadamard Transform Simulation
Implements both reference (loop) and branchless (hardware) FWHT
for cross-validation.
"""

import math

# =============================================================================
# Fixed-width integer helpers (simulates HDL signal width)
# =============================================================================

BIT_WIDTH = 16  # adjust as needed
W = (1 << BIT_WIDTH) - 1


def bit_not(x):
    """Fixed-width bitwise complement. In HDL this is implicit via signal width."""
    return ~x & W

# =============================================================================
# Image data aquesition
# =============================================================================

def load_frame(data_path):
    PIXELS_X = 320
    PIXELS_Y = 240
    with open(data_path, 'rb') as f:
        data = f.read()
        assert len(data) == PIXELS_X * PIXELS_Y

        sample_frame = []
        for y in range(PIXELS_Y):
            row = data[y*PIXELS_X : y*PIXELS_X + PIXELS_X]
            sample_frame.append(row)
    return sample_frame


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


def wht_2D(block):
    # step 1: row pass
    result = []
    for row in block:
        result.append(FWHT(list(row)).run())

    # step 2: column pass
    for c in range(8):
        col = [result[r][c] for r in range(8)]  # extract column
        col = FWHT(col).run()                    # transform it
        for r in range(8):
            result[r][c] = col[r]               # write it back

    return result


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
def diff_frames(frame_a, frame_b):
    diff = []
    for y in range(240):
        row = [int(frame_b[y][x]) - int(frame_a[y][x]) for x in range(320)]
        diff.append(row)
    return diff

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


# =============================================================================
# Adaptive per-block noise estimator
#
# Each block maintains a running RMS estimate of its AC energy. The threshold
# is k*sigma above that estimate. Blocks that are always noisy adapt upward;
# blocks that are always clean stay low. No static calibration required.
#
# alpha: learning rate. 0.05 = ~20 frames to adapt to scene change.
# k:     sigma multiplier. 3.0 = 99.7% of pure noise rejected.
# =============================================================================

class BlockNoiseEstimator:
    def __init__(self, alpha=0.05, k=3.0):
        self.alpha = alpha
        self.k = k
        # initialize conservatively high — adapts down on static scene
        self.noise_est = [[100.0] * (320 // 8) for _ in range(240 // 8)]

    def update_and_threshold(self, by, bx, ac_coeffs):
        idx_y = by // 8
        idx_x = bx // 8

        # RMS of AC coefficients as energy proxy
        rms = math.sqrt(sum(v * v for v in ac_coeffs) / len(ac_coeffs))

        # exponential moving average update
        self.noise_est[idx_y][idx_x] = (
            self.alpha * rms +
            (1 - self.alpha) * self.noise_est[idx_y][idx_x]
        )

        threshold = self.k * self.noise_est[idx_y][idx_x]
        return max(abs(v) for v in ac_coeffs) > threshold


def analyze_compression(diff, keep_n=4, use_adaptive=True, fixed_threshold=100):
    changed_blocks = 0
    total_blocks = 0
    total_coeffs_sent = 0

    estimator = BlockNoiseEstimator() if use_adaptive else None

    for by in range(0, 240, 8):
        for bx in range(0, 320, 8):
            block = [diff[y][bx:bx+8] for y in range(by, by+8)]
            total_blocks += 1

            coeffs = wht_2D(block)

            # skip DC (coeffs[0][0]), only look at AC
            ac_coeffs = [abs(coeffs[r][c]) for r in range(8) for c in range(8) if not (r==0 and c==0)]

            if use_adaptive:
                block_changed = estimator.update_and_threshold(by, bx, ac_coeffs)
            else:
                block_changed = max(ac_coeffs) >= fixed_threshold

            if not block_changed:
                continue

            changed_blocks += 1
            if use_adaptive:
                idx_y = by // 8
                idx_x = bx // 8
                threshold = estimator.k * estimator.noise_est[idx_y][idx_x]
                significant = sum(1 for v in ac_coeffs if v > threshold)
            else:
                significant = sum(1 for v in ac_coeffs if v > fixed_threshold)

            total_coeffs_sent += min(significant, keep_n)

    raw_diff_bytes = 320 * 240
    # each coeff = 1 byte index + 2 bytes value = 3 bytes, plus 3 byte block header
    compressed_bytes = changed_blocks * 3 + total_coeffs_sent * 3

    mode = "adaptive" if use_adaptive else f"fixed threshold={fixed_threshold}"
    print(f"\n  Mode:               {mode}")
    print(f"  Total blocks:       {total_blocks}")
    print(f"  Changed blocks:     {changed_blocks} ({100*changed_blocks/total_blocks:.1f}%)")
    print(f"  Raw diff size:      {raw_diff_bytes} bytes")
    print(f"  Compressed size:    {compressed_bytes} bytes")
    print(f"  Compression ratio:  {raw_diff_bytes/max(compressed_bytes,1):.1f}x")
    print(f"  SF7 FPS:            {683/max(compressed_bytes,1):.2f}")


def main():
    row = 100
    x = 160
    print("Walsh-Hadamard Transform Validation")
    print("=" * 60)

    for exp in range(1, 8):
        N = 1 << exp
        validate(N)

    print("=" * 60)

    print("\nDemo: N=8")
    frame_a = load_frame("../ESP_Ecosystem/esp32cam/docs/testing/artifact/test4/img_0000.raw")
    frame_b = load_frame("../ESP_Ecosystem/esp32cam/docs/testing/artifact/test4/img_0003.raw")
    diff = diff_frames(frame_a, frame_b)
    block = [diff[y][x:x+8] for y in range(row, row+8)]
    coeffs = wht_2D(block)

    print(f"\n{'Row':<4} {'--- Input 8x8 (pixels) ---':<50} {'--- WHT Coefficients ---'}")
    print("-" * 100)
    for r in range(8):
        pixels = [int(b) for b in block[r]]
        coeff_row = coeffs[r]
        px_str = "  ".join(f"{v:4d}" for v in pixels)
        co_str = "  ".join(f"{v:6d}" for v in coeff_row)
        print(f"{r:<4} {px_str}    {co_str}")

    flat = [coeffs[r][c] for r in range(8) for c in range(8)]
    flat_abs = sorted([abs(v) for v in flat], reverse=True)
    print(f"\n  DC coefficient:      {coeffs[0][0]}")
    print(f"  Top 5 magnitudes:    {flat_abs[:5]}")
    print(f"  Bottom 5 magnitudes: {flat_abs[-5:]}")
    total_energy = sum(v*v for v in flat)
    dc_energy = coeffs[0][0] ** 2
    print(f"  DC energy fraction:  {dc_energy/total_energy*100:.1f}%")

    print("\n--- Compression Analysis ---")
    analyze_compression(diff, use_adaptive=True)
    analyze_compression(diff, use_adaptive=False, fixed_threshold=100)

if __name__ == "__main__":
    main()
