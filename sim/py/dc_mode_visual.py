#!/usr/bin/env python3
"""
SDD501 - 4-bit DC mode visual test
Bounding box + bitmap + 4-bit DC + Gaussian reconstruction
Target: ~100 bytes for a small deer
"""

import math
from PIL import Image

WIDTH = 640
HEIGHT = 480
BLOCK_SIZE = 8

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
    return [[b[y][x] - a[y][x] for x in range(WIDTH)] for y in range(HEIGHT)]

def compute_psnr(original, reconstructed):
    mse = 0.0
    for y in range(HEIGHT):
        for x in range(WIDTH):
            d = original[y][x] - reconstructed[y][x]
            mse += d * d
    mse /= (WIDTH * HEIGHT)
    if mse == 0:
        return float('inf')
    return 10 * math.log10(255 * 255 / mse)

def save_frame(frame, path):
    img = Image.new('L', (WIDTH, HEIGHT))
    for y in range(HEIGHT):
        for x in range(WIDTH):
            img.putpixel((x, y), max(0, min(255, frame[y][x])))
    img.save(path)

def ellipse(x, y, ex, ey, rx, ry):
    dx = (x - ex) / rx
    dy = (y - ey) / ry
    return dx*dx + dy*dy

def add_deer(frame, cx, cy):
    new = [row[:] for row in frame]
    for y in range(HEIGHT):
        for x in range(WIDTH):
            delta = 0
            d = ellipse(x, y, cx, cy, 60, 35)
            if d < 1.0: delta = int(80 * (1.0 - d))
            d = ellipse(x, y, cx + 55, cy - 20, 20, 18)
            if d < 1.0: delta = max(delta, int(70 * (1.0 - d)))
            d = ellipse(x, y, cx + 35, cy - 10, 25, 12)
            if d < 1.0: delta = max(delta, int(60 * (1.0 - d)))
            for lx_off, ly_off in [(-30, 35), (-10, 38), (15, 37), (35, 35)]:
                d = ellipse(x, y, cx + lx_off, cy + ly_off, 6, 25)
                if d < 1.0: delta = max(delta, int(50 * (1.0 - d)))
            if delta > 0:
                new[y][x] = max(0, min(255, new[y][x] + delta))
    return new

# =============================================================================
# Gaussian block-level interpolation (smooth the blocky DC reconstruction)
# =============================================================================

def gaussian_smooth_frame(frame, ref, changed_mask, sigma=4.0):
    """Apply Gaussian smoothing only to the changed region of the diff,
    then add back to reference. This smooths the blockiness."""
    # Extract just the diff in changed region
    diff_region = [[0]*WIDTH for _ in range(HEIGHT)]
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if changed_mask[y][x]:
                diff_region[y][x] = frame[y][x] - ref[y][x]

    # Gaussian smooth the diff
    k_half = int(sigma * 2)
    kernel = [math.exp(-0.5 * (i / sigma) ** 2) for i in range(-k_half, k_half + 1)]
    k_sum = sum(kernel)
    kernel = [k / k_sum for k in kernel]

    # Horizontal pass
    temp = [[0.0]*WIDTH for _ in range(HEIGHT)]
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if not changed_mask[y][x]:
                continue
            val = 0.0
            wt = 0.0
            for k in range(-k_half, k_half + 1):
                nx = x + k
                if 0 <= nx < WIDTH and changed_mask[y][nx]:
                    w = kernel[k + k_half]
                    val += w * diff_region[y][nx]
                    wt += w
            if wt > 0:
                temp[y][x] = val / wt

    # Vertical pass
    smoothed = [[0.0]*WIDTH for _ in range(HEIGHT)]
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if not changed_mask[y][x]:
                continue
            val = 0.0
            wt = 0.0
            for k in range(-k_half, k_half + 1):
                ny = y + k
                if 0 <= ny < HEIGHT and changed_mask[ny][x]:
                    w = kernel[k + k_half]
                    val += w * temp[ny][x]
                    wt += w
            if wt > 0:
                smoothed[y][x] = val / wt

    # Reconstruct
    result = [row[:] for row in ref]
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if changed_mask[y][x]:
                result[y][x] = max(0, min(255, ref[y][x] + int(smoothed[y][x])))
    return result

# =============================================================================
# Pipeline modes
# =============================================================================

def run_all_modes(ref, cur, label):
    diff = frame_diff(ref, cur)

    blocks_x = WIDTH // BLOCK_SIZE
    blocks_y = HEIGHT // BLOCK_SIZE

    # Find changed blocks
    changed = []
    for by in range(blocks_y):
        for bx in range(blocks_x):
            has_change = False
            block = []
            for y in range(BLOCK_SIZE):
                row = []
                for x in range(BLOCK_SIZE):
                    val = diff[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x]
                    if abs(val) <= 3: val = 0
                    if abs(val) > 5: has_change = True
                    row.append(val)
                block.append(row)
            if has_change:
                changed.append((bx, by, block))

    # Compute bounding box
    min_bx = min(bx for bx, by, _ in changed)
    max_bx = max(bx for bx, by, _ in changed)
    min_by = min(by for bx, by, _ in changed)
    max_by = max(by for bx, by, _ in changed)
    bbox_w = max_bx - min_bx + 1
    bbox_h = max_by - min_by + 1
    bitmap_bits = bbox_w * bbox_h
    bitmap_bytes = (bitmap_bits + 7) // 8

    # Build changed set for fast lookup
    changed_set = {(bx, by) for bx, by, _ in changed}
    changed_dict = {(bx, by): block for bx, by, block in changed}

    print(f"\n{'='*80}")
    print(f"  {label}")
    print(f"  {len(changed)} changed blocks, bbox: {bbox_w}x{bbox_h} = {bitmap_bits} blocks")
    print(f"{'='*80}")

    # Get DC values for each changed block
    dc_values = {}
    for bx, by, block in changed:
        coeffs = wht_2d(block, BLOCK_SIZE, BLOCK_SIZE)
        dc_values[(bx, by)] = coeffs[0][0]

    # Find DC range for quantization
    dc_min = min(dc_values.values())
    dc_max = max(dc_values.values())
    print(f"  DC range: [{dc_min}, {dc_max}]")

    results = []

    # =====================================================================
    # Mode 1: WHT top-1 packed (baseline)
    # =====================================================================
    packed_b = 3 + len(changed) * (2 + 1 * 2)
    recon1 = [row[:] for row in ref]
    for bx, by, block in changed:
        coeffs = wht_2d(block, BLOCK_SIZE, BLOCK_SIZE)
        flat = sorted(
            [(abs(coeffs[y][x]), y, x, coeffs[y][x])
             for y in range(BLOCK_SIZE) for x in range(BLOCK_SIZE)],
            reverse=True)
        sparse = [[0]*BLOCK_SIZE for _ in range(BLOCK_SIZE)]
        sparse[flat[0][1]][flat[0][2]] = flat[0][3]
        recovered = iwht_2d(sparse, BLOCK_SIZE, BLOCK_SIZE)
        for y in range(BLOCK_SIZE):
            for x in range(BLOCK_SIZE):
                recon1[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x] = max(0, min(255,
                    ref[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x] + recovered[y][x]))
    psnr1 = compute_psnr(cur, recon1)
    save_frame(recon1, f'/home/claude/{label}_wht_top1.png')
    results.append(("WHT top-1 packed", packed_b, psnr1))

    # =====================================================================
    # Mode 2: 8-bit DC + bbox + bitmap (no Gaussian)
    # =====================================================================
    bytes_8bit = 3 + 4 + bitmap_bytes + len(changed) * 1
    recon2 = [row[:] for row in ref]
    for bx, by in changed_set:
        dc = dc_values[(bx, by)]
        # DC coefficient only, quantized to 8 bits
        dc_quant = max(-128, min(127, dc // (BLOCK_SIZE * BLOCK_SIZE)))
        dc_restore = dc_quant * (BLOCK_SIZE * BLOCK_SIZE)
        sparse = [[0]*BLOCK_SIZE for _ in range(BLOCK_SIZE)]
        sparse[0][0] = dc_restore
        recovered = iwht_2d(sparse, BLOCK_SIZE, BLOCK_SIZE)
        for y in range(BLOCK_SIZE):
            for x in range(BLOCK_SIZE):
                recon2[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x] = max(0, min(255,
                    ref[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x] + recovered[y][x]))
    psnr2 = compute_psnr(cur, recon2)
    save_frame(recon2, f'/home/claude/{label}_dc8_raw.png')
    results.append(("8-bit DC bbox (raw)", bytes_8bit, psnr2))

    # =====================================================================
    # Mode 3: 8-bit DC + bbox + Gaussian smooth
    # =====================================================================
    changed_mask = [[False]*WIDTH for _ in range(HEIGHT)]
    for bx, by in changed_set:
        for y in range(BLOCK_SIZE):
            for x in range(BLOCK_SIZE):
                changed_mask[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x] = True

    recon3 = gaussian_smooth_frame(recon2, ref, changed_mask, sigma=5.0)
    psnr3 = compute_psnr(cur, recon3)
    save_frame(recon3, f'/home/claude/{label}_dc8_gaussian.png')
    results.append(("8-bit DC bbox + Gaussian", bytes_8bit, psnr3))

    # =====================================================================
    # Mode 4: 4-bit DC + bbox + bitmap (no Gaussian)
    # =====================================================================
    bytes_4bit = 3 + 4 + bitmap_bytes + (len(changed) + 1) // 2

    # Quantize DC to 4 bits (16 levels)
    dc_abs_max = max(abs(dc_min), abs(dc_max))
    if dc_abs_max == 0: dc_abs_max = 1
    scale = dc_abs_max / 7  # 4 bits signed: -8 to +7

    recon4 = [row[:] for row in ref]
    for bx, by in changed_set:
        dc = dc_values[(bx, by)]
        dc_quant = max(-8, min(7, round(dc / scale / (BLOCK_SIZE * BLOCK_SIZE))))
        dc_restore = int(dc_quant * scale * (BLOCK_SIZE * BLOCK_SIZE))
        sparse = [[0]*BLOCK_SIZE for _ in range(BLOCK_SIZE)]
        sparse[0][0] = dc_restore
        recovered = iwht_2d(sparse, BLOCK_SIZE, BLOCK_SIZE)
        for y in range(BLOCK_SIZE):
            for x in range(BLOCK_SIZE):
                recon4[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x] = max(0, min(255,
                    ref[by*BLOCK_SIZE+y][bx*BLOCK_SIZE+x] + recovered[y][x]))
    psnr4 = compute_psnr(cur, recon4)
    save_frame(recon4, f'/home/claude/{label}_dc4_raw.png')
    results.append(("4-bit DC bbox (raw)", bytes_4bit, psnr4))

    # =====================================================================
    # Mode 5: 4-bit DC + bbox + Gaussian smooth
    # =====================================================================
    recon5 = gaussian_smooth_frame(recon4, ref, changed_mask, sigma=5.0)
    psnr5 = compute_psnr(cur, recon5)
    save_frame(recon5, f'/home/claude/{label}_dc4_gaussian.png')
    results.append(("4-bit DC bbox + Gaussian", bytes_4bit, psnr5))

    # =====================================================================
    # Mode 6: 4-bit DC + bbox + wider Gaussian
    # =====================================================================
    recon6 = gaussian_smooth_frame(recon4, ref, changed_mask, sigma=8.0)
    psnr6 = compute_psnr(cur, recon6)
    save_frame(recon6, f'/home/claude/{label}_dc4_gaussian_wide.png')
    results.append(("4-bit DC bbox + Gaussian σ=8", bytes_4bit, psnr6))

    # Print results
    print(f"\n  {'Mode':<32} | {'Bytes':>7} | {'Ratio':>7} | {'PSNR':>7} | {'SF7 ms':>7} | {'FPS':>5}")
    print(f"  {'-'*32}-+-{'-'*7}-+-{'-'*7}-+-{'-'*7}-+-{'-'*7}-+-{'-'*5}")
    for name, b, p in results:
        r = WIDTH*HEIGHT / b
        tx = b / 683 * 1000
        fps = 683 / b
        print(f"  {name:<32} | {b:>7} | {r:>6.0f}x | {p:>5.1f}dB | {tx:>5.0f}ms | {fps:>5.1f}")

    return results


def main():
    base = '/mnt/user-data/uploads'
    ref = load_raw_frame(f'{base}/img_0000.raw')

    # Small deer
    deer = add_deer(ref, 300, 250)
    run_all_modes(ref, deer, "deer")

    # Save original for reference
    save_frame(deer, '/home/claude/deer_original_ref.png')

    print("\nImages saved.")


if __name__ == "__main__":
    main()
