#!/usr/bin/env python3
"""
SDD501 - Lossy compression experiments on real camera data
Tests:
1. Top-N WHT coefficient selection (keep only largest coefficients)
2. Gaussian blob fitting for diff regions
3. Cosine basis approximation
"""

import math

WIDTH = 640
HEIGHT = 480
BLOCK_SIZE = 8

# =============================================================================
# WHT core
# =============================================================================

def fwht(data):
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

def ifwht(data):
    x = fwht(data)
    N = len(x)
    return [v // N for v in x]

def wht_2d(patch, w, h):
    result = []
    for y in range(h):
        result.append(fwht(patch[y]))
    for x in range(w):
        col = [result[y][x] for y in range(h)]
        col = fwht(col)
        for y in range(h):
            result[y][x] = col[y]
    return result

def iwht_2d(coeffs, w, h):
    result = [row[:] for row in coeffs]
    for x in range(w):
        col = [result[y][x] for y in range(h)]
        col = ifwht(col)
        for y in range(h):
            result[y][x] = col[y]
    for y in range(h):
        result[y] = ifwht(result[y])
    return result

# =============================================================================
# Frame loading and utils
# =============================================================================

def load_raw_frame(path):
    with open(path, 'rb') as f:
        data = f.read()
    frame = []
    for y in range(HEIGHT):
        row = []
        for x in range(WIDTH):
            row.append(data[y * WIDTH + x])
        frame.append(row)
    return frame

def frame_diff(a, b):
    diff = []
    for y in range(HEIGHT):
        row = []
        for x in range(WIDTH):
            row.append(b[y][x] - a[y][x])
        diff.append(row)
    return diff

def compute_psnr(original, reconstructed):
    mse = 0.0
    count = 0
    for y in range(HEIGHT):
        for x in range(WIDTH):
            d = original[y][x] - reconstructed[y][x]
            mse += d * d
            count += 1
    mse /= count
    if mse == 0:
        return float('inf')
    return 10 * math.log10(255 * 255 / mse)

def save_frame_as_pgm(frame, path):
    """Save frame as PGM for easy viewing."""
    with open(path, 'wb') as f:
        f.write(f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode())
        for y in range(HEIGHT):
            for x in range(WIDTH):
                f.write(bytes([max(0, min(255, frame[y][x]))]))

# =============================================================================
# Approach 1: Top-N WHT coefficients per block
# =============================================================================

def topn_block_compress(block, keep_n):
    """WHT an 8x8 block, keep only the top N coefficients by magnitude."""
    coeffs = wht_2d(block, BLOCK_SIZE, BLOCK_SIZE)

    # flatten and sort by magnitude
    flat = []
    for y in range(BLOCK_SIZE):
        for x in range(BLOCK_SIZE):
            flat.append((abs(coeffs[y][x]), y, x, coeffs[y][x]))

    flat.sort(reverse=True)

    # keep only top N
    kept = flat[:keep_n]

    # reconstruct sparse coefficient grid
    sparse = [[0] * BLOCK_SIZE for _ in range(BLOCK_SIZE)]
    for _, y, x, val in kept:
        sparse[y][x] = val

    # inverse WHT
    recovered = iwht_2d(sparse, BLOCK_SIZE, BLOCK_SIZE)

    # encoding cost: per coefficient = 1 byte index + 2 bytes value = 3 bytes
    # plus block header = 3 bytes (bx, by, count)
    byte_cost = 3 + keep_n * 3

    return recovered, byte_cost


def test_topn(ref_path, cur_path, denoise=5, diff_threshold=10):
    """Test top-N compression on real frames."""
    print(f"\n{'='*70}")
    print(f"Top-N WHT Coefficient Selection")
    print(f"  {ref_path} → {cur_path}")
    print(f"{'='*70}")

    ref = load_raw_frame(ref_path)
    cur = load_raw_frame(cur_path)
    diff = frame_diff(ref, cur)

    # find changed blocks
    blocks_x = WIDTH // BLOCK_SIZE
    blocks_y = HEIGHT // BLOCK_SIZE
    changed = []
    for by in range(blocks_y):
        for bx in range(blocks_x):
            has_change = False
            block = []
            for y in range(BLOCK_SIZE):
                row = []
                for x in range(BLOCK_SIZE):
                    val = diff[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x]
                    if abs(val) <= denoise:
                        val = 0
                    if abs(val) > diff_threshold:
                        has_change = True
                    row.append(val)
                block.append(row)
            if has_change:
                changed.append((bx, by, block))

    print(f"  Changed blocks: {len(changed)} / {blocks_x * blocks_y}")
    print()
    print(f"  {'Keep N':>6} | {'Bytes':>8} | {'vs Raw':>7} | {'vs Frame':>8} | {'PSNR':>8} | {'TX ms':>8} | {'FPS':>5}")
    print(f"  {'-'*6}-+-{'-'*8}-+-{'-'*7}-+-{'-'*8}-+-{'-'*8}-+-{'-'*8}-+-{'-'*5}")

    raw_frame = WIDTH * HEIGHT

    for keep_n in [1, 2, 3, 4, 6, 8, 12, 16, 32]:
        total_bytes = 2  # header
        recon = [row[:] for row in ref]

        for bx, by, block in changed:
            recovered, cost = topn_block_compress(block, keep_n)
            total_bytes += cost

            for y in range(BLOCK_SIZE):
                for x in range(BLOCK_SIZE):
                    px = bx * BLOCK_SIZE + x
                    py = by * BLOCK_SIZE + y
                    val = ref[py][px] + recovered[y][x]
                    recon[py][px] = max(0, min(255, val))

        psnr = compute_psnr(cur, recon)
        raw_patch = len(changed) * 64
        ratio_patch = raw_patch / max(total_bytes, 1)
        ratio_frame = raw_frame / max(total_bytes, 1)
        tx_ms = total_bytes / 683 * 1000
        fps = 683 / max(total_bytes, 1)

        print(f"  {keep_n:>6} | {total_bytes:>8} | {ratio_patch:>6.2f}x | {ratio_frame:>7.1f}x | {psnr:>6.2f}dB | {tx_ms:>6.0f}ms | {fps:>5.1f}")


# =============================================================================
# Approach 2: Gaussian blob fitting
# =============================================================================

def fit_gaussian_to_diff(diff, threshold=10):
    """Fit a single Gaussian blob to the diff region.
    Returns (cx, cy, sigma_x, sigma_y, amplitude) and byte cost."""

    # find weighted centroid and spread of changed pixels
    sum_w = 0
    sum_wx = 0
    sum_wy = 0
    sum_wxx = 0
    sum_wyy = 0
    max_val = 0

    for y in range(HEIGHT):
        for x in range(WIDTH):
            w = abs(diff[y][x])
            if w > threshold:
                sum_w += w
                sum_wx += w * x
                sum_wy += w * y
                sum_wxx += w * x * x
                sum_wyy += w * y * y
                if w > max_val:
                    max_val = w

    if sum_w == 0:
        return None

    cx = sum_wx / sum_w
    cy = sum_wy / sum_w
    var_x = sum_wxx / sum_w - cx * cx
    var_y = sum_wyy / sum_w - cy * cy
    sigma_x = math.sqrt(max(var_x, 1))
    sigma_y = math.sqrt(max(var_y, 1))

    # determine sign (is the diff mostly positive or negative?)
    sum_signed = 0
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if abs(diff[y][x]) > threshold:
                sum_signed += diff[y][x]
    sign = 1 if sum_signed >= 0 else -1

    amplitude = sign * max_val

    # encoding: cx(2) + cy(2) + sigma_x(2) + sigma_y(2) + amplitude(2) = 10 bytes
    byte_cost = 10

    return (cx, cy, sigma_x, sigma_y, amplitude), byte_cost


def reconstruct_gaussian(ref, params):
    """Reconstruct frame by adding Gaussian blob to reference."""
    cx, cy, sigma_x, sigma_y, amplitude = params
    recon = [row[:] for row in ref]

    for y in range(HEIGHT):
        for x in range(WIDTH):
            dx = (x - cx) / sigma_x
            dy = (y - cy) / sigma_y
            g = amplitude * math.exp(-0.5 * (dx*dx + dy*dy))
            recon[y][x] = max(0, min(255, recon[y][x] + int(g)))

    return recon


def fit_multi_gaussian(diff, n_gaussians=3, threshold=10):
    """Iteratively fit multiple Gaussians to residual."""
    residual = [row[:] for row in diff]
    all_params = []
    total_bytes = 2  # header: count

    for _ in range(n_gaussians):
        result = fit_gaussian_to_diff(residual, threshold)
        if result is None:
            break

        params, cost = result
        all_params.append(params)
        total_bytes += cost

        # subtract fitted Gaussian from residual
        cx, cy, sigma_x, sigma_y, amplitude = params
        for y in range(HEIGHT):
            for x in range(WIDTH):
                dx = (x - cx) / sigma_x
                dy = (y - cy) / sigma_y
                g = amplitude * math.exp(-0.5 * (dx*dx + dy*dy))
                residual[y][x] -= int(g)

    return all_params, total_bytes


def test_gaussian(ref_path, cur_path):
    """Test Gaussian blob compression."""
    print(f"\n{'='*70}")
    print(f"Gaussian Blob Fitting")
    print(f"  {ref_path} → {cur_path}")
    print(f"{'='*70}")

    ref = load_raw_frame(ref_path)
    cur = load_raw_frame(cur_path)
    diff = frame_diff(ref, cur)

    print(f"  {'Gaussians':>9} | {'Bytes':>8} | {'vs Frame':>8} | {'PSNR':>8} | {'TX ms':>8} | {'FPS':>5}")
    print(f"  {'-'*9}-+-{'-'*8}-+-{'-'*8}-+-{'-'*8}-+-{'-'*8}-+-{'-'*5}")

    for n_gauss in [1, 2, 3, 5, 8, 12, 20, 50]:
        params_list, total_bytes = fit_multi_gaussian(diff, n_gauss)

        # reconstruct
        recon = [row[:] for row in ref]
        for params in params_list:
            cx, cy, sigma_x, sigma_y, amplitude = params
            for y in range(HEIGHT):
                for x in range(WIDTH):
                    dx = (x - cx) / sigma_x
                    dy = (y - cy) / sigma_y
                    g = amplitude * math.exp(-0.5 * (dx*dx + dy*dy))
                    recon[y][x] = max(0, min(255, recon[y][x] + int(g)))

        psnr = compute_psnr(cur, recon)
        ratio = WIDTH * HEIGHT / max(total_bytes, 1)
        tx_ms = total_bytes / 683 * 1000
        fps = 683 / max(total_bytes, 1)

        print(f"  {len(params_list):>9} | {total_bytes:>8} | {ratio:>7.1f}x | {psnr:>6.2f}dB | {tx_ms:>6.0f}ms | {fps:>5.1f}")


# =============================================================================
# Approach 3: Hybrid — Gaussian for gross shape + WHT for residual detail
# =============================================================================

def test_hybrid(ref_path, cur_path, n_gaussians=5, keep_n=4, denoise=5, diff_threshold=10):
    """Gaussian for coarse shape, then WHT top-N on residual."""
    print(f"\n{'='*70}")
    print(f"Hybrid: {n_gaussians} Gaussians + Top-{keep_n} WHT residual")
    print(f"  {ref_path} → {cur_path}")
    print(f"{'='*70}")

    ref = load_raw_frame(ref_path)
    cur = load_raw_frame(cur_path)
    diff = frame_diff(ref, cur)

    # Phase 1: Gaussian fitting
    params_list, gauss_bytes = fit_multi_gaussian(diff, n_gaussians)

    # compute residual after Gaussian subtraction
    residual = [row[:] for row in diff]
    for params in params_list:
        cx, cy, sigma_x, sigma_y, amplitude = params
        for y in range(HEIGHT):
            for x in range(WIDTH):
                dx = (x - cx) / sigma_x
                dy = (y - cy) / sigma_y
                g = amplitude * math.exp(-0.5 * (dx*dx + dy*dy))
                residual[y][x] -= int(g)

    # Phase 2: WHT on residual blocks
    blocks_x = WIDTH // BLOCK_SIZE
    blocks_y = HEIGHT // BLOCK_SIZE
    changed = []
    for by in range(blocks_y):
        for bx in range(blocks_x):
            has_change = False
            block = []
            for y in range(BLOCK_SIZE):
                row = []
                for x in range(BLOCK_SIZE):
                    val = residual[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x]
                    if abs(val) <= denoise:
                        val = 0
                    if abs(val) > diff_threshold:
                        has_change = True
                    row.append(val)
                block.append(row)
            if has_change:
                changed.append((bx, by, block))

    wht_bytes = 2  # header
    recon = [row[:] for row in ref]

    # add Gaussians to reconstruction
    for params in params_list:
        cx, cy, sigma_x, sigma_y, amplitude = params
        for y in range(HEIGHT):
            for x in range(WIDTH):
                dx = (x - cx) / sigma_x
                dy = (y - cy) / sigma_y
                g = amplitude * math.exp(-0.5 * (dx*dx + dy*dy))
                recon[y][x] = max(0, min(255, recon[y][x] + int(g)))

    # add WHT residual patches
    for bx, by, block in changed:
        recovered, cost = topn_block_compress(block, keep_n)
        wht_bytes += cost
        for y in range(BLOCK_SIZE):
            for x in range(BLOCK_SIZE):
                px = bx * BLOCK_SIZE + x
                py = by * BLOCK_SIZE + y
                val = recon[py][px] + recovered[y][x]
                recon[py][px] = max(0, min(255, val))

    total_bytes = gauss_bytes + wht_bytes
    psnr = compute_psnr(cur, recon)
    ratio = WIDTH * HEIGHT / max(total_bytes, 1)
    tx_ms = total_bytes / 683 * 1000
    fps = 683 / max(total_bytes, 1)

    print(f"  Gaussian bytes: {gauss_bytes}")
    print(f"  WHT residual blocks: {len(changed)}")
    print(f"  WHT bytes: {wht_bytes}")
    print(f"  Total bytes: {total_bytes}")
    print(f"  Compression vs frame: {ratio:.1f}x")
    print(f"  PSNR: {psnr:.2f} dB")
    print(f"  TX time: {tx_ms:.0f} ms")
    print(f"  FPS: {fps:.1f}")


# =============================================================================
# Main
# =============================================================================

def main():
    base = '/mnt/user-data/uploads'

    # Static pair (noise-dominated)
    print("\n" + "=" * 70)
    print("STATIC SCENE (frames 0 → 1, mostly sensor noise)")
    print("=" * 70)
    test_topn(f'{base}/img_0000.raw', f'{base}/img_0001.raw')
    test_gaussian(f'{base}/img_0000.raw', f'{base}/img_0001.raw')

    # Motion pair
    print("\n" + "=" * 70)
    print("MOTION SCENE (frames 0 → 2, camera moved)")
    print("=" * 70)
    test_topn(f'{base}/img_0000.raw', f'{base}/img_0002.raw')
    test_gaussian(f'{base}/img_0000.raw', f'{base}/img_0002.raw')

    # Hybrid on motion
    test_hybrid(f'{base}/img_0000.raw', f'{base}/img_0002.raw',
                n_gaussians=10, keep_n=4)

    # Adjacent frames with less motion (2 → 3)
    print("\n" + "=" * 70)
    print("ADJACENT FRAMES (frames 2 → 3)")
    print("=" * 70)
    test_topn(f'{base}/img_0002.raw', f'{base}/img_0003.raw')
    test_gaussian(f'{base}/img_0002.raw', f'{base}/img_0003.raw')
    test_hybrid(f'{base}/img_0002.raw', f'{base}/img_0003.raw',
                n_gaussians=10, keep_n=4)


if __name__ == "__main__":
    main()
