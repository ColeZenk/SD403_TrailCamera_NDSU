"""
Motion-compensated compression pipeline.

Block matching ME + inter/intra classification + residual coding.
"""

from wht_core import wht_2D, iwht_2D
from codec import sequency_mask, _asr, _q_step_to_shift

import math


def motion_estimate(frame_a, frame_b, search_radius=8, sad_threshold=256):
    """
    Block matching motion estimation. O(blocks x search_window).

    For each 8x8 block in frame_b, search frame_a within search_radius pixels
    using Sum of Absolute Differences (SAD). Returns:
      mv_field: dict (by, bx) -> (dy, dx, best_sad)
      fa_np, fb_np: numpy arrays (240x320) for downstream use
    """
    import numpy as np
    fa_np = np.array([[float(frame_a[y][x]) for x in range(320)]
                       for y in range(240)], dtype=np.float32)
    fb_np = np.array([[float(frame_b[y][x]) for x in range(320)]
                       for y in range(240)], dtype=np.float32)

    mv_field = {}
    for by in range(0, 240, 8):
        for bx in range(0, 320, 8):
            fb_block  = fb_np[by:by+8, bx:bx+8]
            best_sad  = float('inf')
            best_dy, best_dx = 0, 0

            for dy in range(-search_radius, search_radius + 1):
                ry = by + dy
                if ry < 0 or ry + 8 > 240:
                    continue
                for dx in range(-search_radius, search_radius + 1):
                    rx = bx + dx
                    if rx < 0 or rx + 8 > 320:
                        continue
                    sad = float(np.sum(np.abs(fa_np[ry:ry+8, rx:rx+8] - fb_block)))
                    if sad < best_sad:
                        best_sad  = sad
                        best_dy, best_dx = dy, dx

            mv_field[(by, bx)] = (best_dy, best_dx, best_sad)

    return mv_field, fa_np, fb_np


def encode_frame_mc(mv_field, fa_np, fb_np,
                    q_step=32, top_k=5, seq_thresh=2,
                    block_threshold=400, sad_threshold=256):
    """
    Motion-compensated encoder. Fixed-point quantization via arithmetic shift.

    Returns (total_bytes, mv_blocks, intra_blocks, mv_residuals).
    """
    q_shift      = _q_step_to_shift(q_step)
    total_bytes  = 6   # frame header: n_mv(2) + n_intra(2) + n_mv_residual(2)
    mv_blocks    = []
    intra_blocks = []
    mv_residuals = []

    for by in range(0, 240, 8):
        for bx in range(0, 320, 8):
            best_dy, best_dx, best_sad = mv_field[(by, bx)]

            if best_sad < sad_threshold:
                # Inter block — transmit motion vector
                mv_blocks.append((by, bx, best_dy, best_dx))
                total_bytes += 4   # bx(1) by(1) dx(1) dy(1)

                # Check residual
                ry, rx   = by + best_dy, bx + best_dx
                residual = [[float(fb_np[by+r, bx+c]) - float(fa_np[ry+r, rx+c])
                             for c in range(8)] for r in range(8)]
                coeffs   = wht_2D(residual)
                masked   = sequency_mask(coeffs, seq_thresh)
                ac_max   = max(abs(masked[r][c])
                               for r in range(8) for c in range(8)
                               if not (r == 0 and c == 0))

                if ac_max >= block_threshold:
                    attenuated = [[0]*8 for _ in range(8)]
                    for r in range(8):
                        for c in range(8):
                            v = masked[r][c]
                            if abs(v) <= block_threshold:
                                attenuated[r][c] = 0
                            else:
                                sign = 1 if v > 0 else -1
                                attenuated[r][c] = sign * (abs(v) - block_threshold)
                    quantized = [(_asr(attenuated[r][c], q_shift), r, c)
                                 for r in range(8) for c in range(8)]
                    nonzero   = sorted([(q, r, c) for q, r, c in quantized if q != 0],
                                       key=lambda x: abs(x[0]), reverse=True)
                    selected  = nonzero[:top_k]
                    if selected:
                        mv_residuals.append((by, bx, selected))
                        total_bytes += 2 + len(selected) + 1
            else:
                # Intra block — WHT of raw diff
                diff_block = [[float(fb_np[by+r, bx+c]) - float(fa_np[by+r, bx+c])
                               for c in range(8)] for r in range(8)]
                coeffs = wht_2D(diff_block)
                masked = sequency_mask(coeffs, seq_thresh)
                ac_max = max(abs(masked[r][c])
                             for r in range(8) for c in range(8)
                             if not (r == 0 and c == 0))
                if ac_max < block_threshold:
                    continue
                attenuated = [[0]*8 for _ in range(8)]
                for r in range(8):
                    for c in range(8):
                        v = masked[r][c]
                        if abs(v) <= block_threshold:
                            attenuated[r][c] = 0
                        else:
                            sign = 1 if v > 0 else -1
                            attenuated[r][c] = sign * (abs(v) - block_threshold)
                quantized = [(_asr(attenuated[r][c], q_shift), r, c)
                             for r in range(8) for c in range(8)]
                nonzero   = sorted([(q, r, c) for q, r, c in quantized if q != 0],
                                   key=lambda x: abs(x[0]), reverse=True)
                selected  = nonzero[:top_k]
                if not selected:
                    continue
                intra_blocks.append((by, bx, selected))
                total_bytes += 2 + len(selected) + 1

    return min(total_bytes, 76800), mv_blocks, intra_blocks, mv_residuals


def decode_frame_mc(frame_a, mv_blocks, intra_blocks, mv_residuals,
                    q_step=32, gauss_sigma=4.0):
    """
    Motion-compensated decoder.

    1. Start from reference frame A.
    2. Warp MC blocks from A using motion vectors.
    3. Accumulate residual diff (MV residuals + intra patches).
    4. Gaussian smooth the diff, add to warped frame.
    """
    import numpy as np
    from scipy.ndimage import gaussian_filter

    q_shift = _q_step_to_shift(q_step)
    fa_np = np.array([[float(frame_a[y][x]) for x in range(320)]
                       for y in range(240)], dtype=np.float64)
    recon = fa_np.copy()

    for by, bx, dy, dx in mv_blocks:
        ry, rx = by + dy, bx + dx
        recon[by:by+8, bx:bx+8] = fa_np[ry:ry+8, rx:rx+8]

    diff_recon = np.zeros((240, 320), dtype=np.float64)

    for by, bx, selected in mv_residuals:
        coeffs = [[0.0] * 8 for _ in range(8)]
        for q, r, c in selected:
            coeffs[r][c] = float(q << q_shift)
        patch = iwht_2D(coeffs)
        for r in range(8):
            for c in range(8):
                diff_recon[by+r, bx+c] = patch[r][c]

    for by, bx, selected in intra_blocks:
        coeffs = [[0.0] * 8 for _ in range(8)]
        for q, r, c in selected:
            coeffs[r][c] = float(q << q_shift)
        patch = iwht_2D(coeffs)
        for r in range(8):
            for c in range(8):
                diff_recon[by+r, bx+c] = patch[r][c]

    if gauss_sigma > 0:
        diff_recon = gaussian_filter(diff_recon, sigma=gauss_sigma)

    return np.clip(recon + diff_recon, 0, 255)


def encode_decode_mc(frame_a, frame_b, q_step=32, top_k=5, seq_thresh=2,
                     block_threshold=400, gauss_sigma=4.0,
                     search_radius=8, sad_threshold=256):
    """
    Full motion-compensated round-trip: estimate -> encode -> decode -> PSNR.
    """
    import numpy as np

    mv_field, fa_np, fb_np = motion_estimate(
        frame_a, frame_b, search_radius, sad_threshold)

    total_bytes, mv_blocks, intra_blocks, mv_residuals = encode_frame_mc(
        mv_field, fa_np, fb_np,
        q_step=q_step, top_k=top_k, seq_thresh=seq_thresh,
        block_threshold=block_threshold, sad_threshold=sad_threshold)

    recon = decode_frame_mc(
        frame_a, mv_blocks, intra_blocks, mv_residuals,
        q_step=q_step, gauss_sigma=gauss_sigma)

    mse  = float(np.mean((fb_np.astype(np.float64) - recon) ** 2))
    psnr = 10 * math.log10(255**2 / mse) if mse > 0 else float('inf')

    n_changed = len(mv_blocks) + len(intra_blocks)
    return {
        'compressed_bytes': total_bytes,
        'ratio':            76800 / max(total_bytes, 1),
        'sf7_fps':          683   / max(total_bytes, 1),
        'psnr':             psnr,
        'changed_blocks':   n_changed,
        'mv_blocks':        len(mv_blocks),
        'intra_blocks':     len(intra_blocks),
        'mv_residuals':     len(mv_residuals),
        'pct_changed':      100 * n_changed / 1200,
        'recon':            recon,
        'fb_np':            fb_np.astype(np.float64),
    }


def analyze_frame_sequence_mc(frames, q_step=32, top_k=5, seq_thresh=2,
                               block_threshold=400, gauss_sigma=4.0,
                               search_radius=8, sad_threshold=256):
    results = []
    for fi in range(len(frames) - 1):
        name_a, fa = frames[fi]
        name_b, fb = frames[fi + 1]
        r = encode_decode_mc(fa, fb, q_step=q_step, top_k=top_k,
                             seq_thresh=seq_thresh,
                             block_threshold=block_threshold,
                             gauss_sigma=gauss_sigma,
                             search_radius=search_radius,
                             sad_threshold=sad_threshold)
        r['pair'] = f"{name_a}->>{name_b}"
        results.append(r)
    return results
