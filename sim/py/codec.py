"""
Codec — encode/decode pipeline for WHT compression.

Transmitter: diff -> WHT -> sequency mask -> quantize -> top-K -> packets
Receiver:    dequantize -> IWHT -> Gaussian smooth -> add to reference
"""

import math

from wht_core import wht_2D, iwht_2D


# =============================================================================
# Frame diff + sequency mask
# =============================================================================

def diff_frames(frame_a, frame_b):
    return [[int(frame_b[y][x]) - int(frame_a[y][x]) for x in range(320)]
            for y in range(240)]


def sequency_mask(coeffs, seq_thresh):
    """
    Walsh-domain low-pass filter applied after WHT.

    Zero all coefficients where r + c > seq_thresh.
    Walsh basis functions are sequency-ordered — r+c is the sequency index,
    mapping directly to spatial frequency band (same intuition as DCT).

    seq_thresh=0 : DC only          (1 coeff)
    seq_thresh=1 : + 1st order      (3 coeffs)
    seq_thresh=2 : + 2nd order      (6 coeffs)  <- default
    seq_thresh=3 : + 3rd order      (10 coeffs)

    FPGA: single comparator on (r, c) address registers after butterfly.
    """
    return [[coeffs[r][c] if (r + c) <= seq_thresh else 0
             for c in range(8)] for r in range(8)]


# =============================================================================
# Noise floor characterisation
# =============================================================================

def noise_floor_stats(frames):
    """Per-block p95 of max AC WHT coefficient across all consecutive pairs."""
    block_max_ac = [[[] for _ in range(40)] for _ in range(30)]
    for fi in range(len(frames) - 1):
        _, fa = frames[fi]
        _, fb = frames[fi + 1]
        diff = diff_frames(fa, fb)
        for by in range(0, 240, 8):
            for bx in range(0, 320, 8):
                block  = [diff[y][bx:bx+8] for y in range(by, by+8)]
                coeffs = wht_2D(block)
                ac     = [abs(coeffs[r][c])
                          for r in range(8) for c in range(8)
                          if not (r == 0 and c == 0)]
                block_max_ac[by//8][bx//8].append(max(ac))

    noise_floor = [
        [sorted(block_max_ac[r][c])[int(len(block_max_ac[r][c]) * 0.95)]
         for c in range(40)]
        for r in range(30)
    ]
    flat = sorted(noise_floor[r][c] for r in range(30) for c in range(40))
    return {
        'p25': flat[300], 'p50': flat[600],
        'p75': flat[900], 'p95': flat[1140], 'max': flat[-1],
    }


# =============================================================================
# Encode / Decode
# =============================================================================

def _asr(v, shift):
    """Arithmetic right shift — matches Verilog >>> operator."""
    if v >= 0:
        return v >> shift
    else:
        return -((-v) >> shift)


def _q_step_to_shift(q_step):
    """Convert q_step (must be power of 2) to shift count for FPGA >>>."""
    assert q_step > 0 and (q_step & (q_step - 1)) == 0, \
        f"q_step must be power of 2, got {q_step}"
    return q_step.bit_length() - 1


def encode_frame(diff, q_step=32, top_k=5, seq_thresh=2, block_threshold=400):
    """
    Encoder: WHT -> sequency mask -> quantize -> patch packets.
    Fixed-point integer arithmetic throughout — mirrors FPGA datapath.

    q_step must be a power of 2. Quantization is arithmetic right shift.
    Returns (byte_cost, encoded_blocks).
    encoded_blocks: list of (by, bx, [(q, r, c), ...])
    """
    q_shift     = _q_step_to_shift(q_step)
    total_bytes = 2   # frame header: n_blocks uint16
    encoded     = []

    for by in range(0, 240, 8):
        for bx in range(0, 320, 8):
            block  = [diff[y][bx:bx+8] for y in range(by, by+8)]
            coeffs = wht_2D(block)
            masked = sequency_mask(coeffs, seq_thresh)

            ac_max = max(abs(masked[r][c])
                         for r in range(8) for c in range(8)
                         if not (r == 0 and c == 0))
            if ac_max < block_threshold:
                continue

            # Soft threshold: integer subtract toward zero
            attenuated = [[0]*8 for _ in range(8)]
            for r in range(8):
                for c in range(8):
                    v = masked[r][c]
                    mag = abs(v)
                    if mag <= block_threshold:
                        attenuated[r][c] = 0
                    else:
                        sign = 1 if v > 0 else -1
                        attenuated[r][c] = sign * (mag - block_threshold)

            # Quantize via arithmetic right shift (matches FPGA >>>)
            quantized = [(_asr(attenuated[r][c], q_shift), r, c)
                         for r in range(8) for c in range(8)]
            nonzero   = sorted([(q, r, c) for q, r, c in quantized if q != 0],
                               key=lambda x: abs(x[0]), reverse=True)
            selected  = nonzero[:top_k]

            if not selected:
                continue

            total_bytes += 2 + len(selected) + 1   # coords + coeffs + EOB
            encoded.append((by, bx, selected))

    return min(total_bytes, 76800), encoded


def decode_frame(frame_a, encoded, q_step=32, gauss_sigma=4.0):
    """
    Decoder: dequantize -> IWHT -> Gaussian smooth -> add to frame_A.

    Dequantize via left shift (q << q_shift), inverse of encode.
    frame_a : list of lists (240x320)
    encoded : list of (by, bx, [(q, r, c), ...]) from encode_frame
    Returns reconstructed frame_B as numpy array (240x320).
    """
    import numpy as np
    from scipy.ndimage import gaussian_filter

    q_shift    = _q_step_to_shift(q_step)
    diff_recon = np.zeros((240, 320), dtype=float)

    for by, bx, selected in encoded:
        coeffs = [[0.0] * 8 for _ in range(8)]
        for q, r, c in selected:
            coeffs[r][c] = float(q << q_shift)
        patch = iwht_2D(coeffs)
        for r in range(8):
            for c in range(8):
                diff_recon[by + r, bx + c] = patch[r][c]

    if gauss_sigma > 0:
        diff_recon = gaussian_filter(diff_recon, sigma=gauss_sigma)

    fa_np = np.array([[float(frame_a[y][x]) for x in range(320)]
                       for y in range(240)])
    return np.clip(fa_np + diff_recon, 0, 255)


def encode_decode(frame_a, frame_b, q_step=32, top_k=5, seq_thresh=2,
                  block_threshold=400, gauss_sigma=4.0):
    """
    Full round-trip: encode -> decode -> PSNR.
    Returns metrics dict including reconstructed frame.
    """
    import numpy as np

    diff                    = diff_frames(frame_a, frame_b)
    sent_bytes, encoded     = encode_frame(diff, q_step=q_step, top_k=top_k,
                                           seq_thresh=seq_thresh,
                                           block_threshold=block_threshold)
    recon                   = decode_frame(frame_a, encoded, q_step=q_step,
                                           gauss_sigma=gauss_sigma)
    fb_np = np.array([[float(frame_b[y][x]) for x in range(320)]
                       for y in range(240)])
    mse  = float(np.mean((fb_np - recon) ** 2))
    psnr = 10 * math.log10(255**2 / mse) if mse > 0 else float('inf')

    return {
        'compressed_bytes': sent_bytes,
        'ratio':            76800 / max(sent_bytes, 1),
        'sf7_fps':          683   / max(sent_bytes, 1),
        'psnr':             psnr,
        'changed_blocks':   len(encoded),
        'pct_changed':      100 * len(encoded) / 1200,
        'recon':            recon,
        'fb_np':            fb_np,
    }


def analyze_frame_sequence(frames, q_step=32, top_k=5, seq_thresh=2,
                            block_threshold=400, gauss_sigma=4.0):
    results = []
    for fi in range(len(frames) - 1):
        name_a, fa = frames[fi]
        name_b, fb = frames[fi + 1]
        r = encode_decode(fa, fb, q_step=q_step, top_k=top_k,
                          seq_thresh=seq_thresh,
                          block_threshold=block_threshold,
                          gauss_sigma=gauss_sigma)
        r['pair']    = f"{name_a}->>{name_b}"
        r['sf7_fps'] = r['sf7_fps']
        results.append(r)
    return results
