#!/usr/bin/env python3
"""
SDD501 - Walsh-Hadamard Transform Simulation
Implements both reference (loop) and branchless (hardware) FWHT
for cross-validation.

Compression pipeline:
  - Box LPF on diff before WHT (kills high-freq sensor noise)
  - Quantization (Q_step)
  - Top-K coefficient ceiling (perceptual hard cap)
  - Adaptive mode selector: patch vs DC+Gaussian, sends smaller

Run with --report to generate a PDF test report.
"""

import math
import os
import glob
import argparse
from datetime import datetime

# =============================================================================
# Fixed-width integer helpers (simulates HDL signal width)
# =============================================================================

BIT_WIDTH = 16
W = (1 << BIT_WIDTH) - 1


def bit_not(x):
    """Fixed-width bitwise complement. In HDL this is implicit via signal width."""
    return ~x & W


# =============================================================================
# Image data acquisition
# =============================================================================

def load_frame(data_path):
    PIXELS_X = 320
    PIXELS_Y = 240
    with open(data_path, 'rb') as f:
        data = f.read()
        assert len(data) == PIXELS_X * PIXELS_Y, \
            f"Expected {PIXELS_X*PIXELS_Y}, got {len(data)} in {data_path}"
        sample_frame = []
        for y in range(PIXELS_Y):
            row = data[y*PIXELS_X : y*PIXELS_X + PIXELS_X]
            sample_frame.append(row)
    return sample_frame


def load_frame_dir(directory):
    """Load all img_NNNN.raw files from a directory, sorted."""
    paths = sorted(glob.glob(os.path.join(directory, 'img_*.raw')))
    frames = []
    for p in paths:
        try:
            frames.append((os.path.basename(p), load_frame(p)))
        except AssertionError as e:
            print(f"  Skipping {p}: {e}")
    return frames


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
        index.x[index.i + index.j]               = a + b
        index.x[index.i + index.j + index.stride] = a - b

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
    import random
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


# =============================================================================
# Frame diff + LPF
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
    Low sequency = low spatial frequency = coarse structure.
    High sequency = high spatial frequency = noise, fine texture.

    seq_thresh=0 : DC only          (1 coeff)
    seq_thresh=1 : + 1st order      (3 coeffs)
    seq_thresh=2 : + 2nd order      (6 coeffs)  <- default
    seq_thresh=3 : + 3rd order      (10 coeffs)

    FPGA: single comparator on (r, c) address registers after butterfly.
    No line buffers, no inter-block memory, zero multipliers.
    Entire filter synthesizes to a handful of LUTs.
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
# Compression — full pipeline
#
# TRANSMITTER (FPGA + ESP32):
#   1. diff = frame_B - frame_A              inter-frame delta
#   2. WHT per 8x8 block                     Walsh transform
#   3. sequency_mask(seq_thresh)             Walsh-domain LPF: zero r+c > thresh
#   4. quantize coefficients by q_step       signed byte per coeff
#   5. skip blocks below block_threshold     no motion
#   6. transmit nonzero masked coeffs up to top_k, sorted by magnitude
#
# PACKET FORMAT per changed block:
#   [1 byte] bx   block x coord (0-39)
#   [1 byte] by   block y coord (0-29)
#   [1 byte each] quantized coefficients, signed, sorted by magnitude desc
#   [1 byte] 0x00 EOB marker
#   Frame header: [2 bytes] n_blocks uint16
#
# RECEIVER (ESP32 / phone):
#   1. dequantize: coeff * q_step
#   2. IWHT per block -> spatial diff patch
#   3. Gaussian smooth reconstructed diff (hides block boundary artifacts)
#   4. frame_B_recon = clip(frame_A + smoothed_diff, 0, 255)
# =============================================================================

def encode_frame(diff, q_step=32, top_k=5, seq_thresh=2, block_threshold=400):
    """
    Encoder: WHT -> sequency mask -> quantize -> patch packets.

    Returns (byte_cost, encoded_blocks).
    encoded_blocks: list of (by, bx, [(q, r, c), ...])
    """
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

            quantized = [(round(masked[r][c] / q_step), r, c)
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

    frame_a : list of lists (240x320)
    encoded : list of (by, bx, [(q, r, c), ...]) from encode_frame
    Returns reconstructed frame_B as numpy array (240x320).
    """
    import numpy as np
    from scipy.ndimage import gaussian_filter

    diff_recon = np.zeros((240, 320), dtype=float)

    for by, bx, selected in encoded:
        coeffs = [[0.0] * 8 for _ in range(8)]
        for q, r, c in selected:
            coeffs[r][c] = float(q * q_step)
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


# =============================================================================
# Sequence analysis
# =============================================================================

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



# =============================================================================
# PDF report
# =============================================================================

def generate_report(val_results, seq_results, noise_stats,
                    frame_dir, cfg, output_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from reportlab.lib.pagesizes import letter
    from reportlab.lib import colors
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units import inch
    from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer,
                                    Table, TableStyle, PageBreak, Image,
                                    HRFlowable)
    from reportlab.lib.enums import TA_CENTER

    doc    = SimpleDocTemplate(output_path, pagesize=letter,
                               leftMargin=0.75*inch, rightMargin=0.75*inch,
                               topMargin=0.75*inch, bottomMargin=0.75*inch)
    styles = getSampleStyleSheet()
    title_style = ParagraphStyle('Title', parent=styles['Title'],
                                 fontSize=18, spaceAfter=6, alignment=TA_CENTER)
    sub_style   = ParagraphStyle('Sub', parent=styles['Normal'],
                                 fontSize=9, textColor=colors.grey,
                                 alignment=TA_CENTER, spaceAfter=16)
    h1_style    = ParagraphStyle('H1', parent=styles['Heading1'],
                                 fontSize=13, spaceBefore=14, spaceAfter=6,
                                 textColor=colors.HexColor('#1a1a2e'))
    body_style  = styles['Normal']

    def tbl():
        return TableStyle([
            ('BACKGROUND',    (0,0),(-1,0), colors.HexColor('#1a1a2e')),
            ('TEXTCOLOR',     (0,0),(-1,0), colors.white),
            ('FONTNAME',      (0,0),(-1,0), 'Helvetica-Bold'),
            ('FONTSIZE',      (0,0),(-1,-1), 8.5),
            ('ALIGN',         (0,0),(-1,-1), 'CENTER'),
            ('ROWBACKGROUNDS',(0,1),(-1,-1),
             [colors.white, colors.HexColor('#f0f4ff')]),
            ('GRID',          (0,0),(-1,-1), 0.4, colors.HexColor('#cccccc')),
            ('TOPPADDING',    (0,0),(-1,-1), 4),
            ('BOTTOMPADDING', (0,0),(-1,-1), 4),
        ])

    story = []

    # Cover
    story.append(Spacer(1, 0.4*inch))
    story.append(Paragraph("Trail Camera IV — WHT Compression Test Report", title_style))
    n_kept = sum(1 for r in range(8) for c in range(8) if r+c <= cfg['seq_thresh'])
    story.append(Paragraph(
        f"SDD501 | {datetime.now().strftime('%Y-%m-%d %H:%M')} | "
        f"Dataset: {os.path.basename(frame_dir) or frame_dir} | "
        f"Q={cfg['q_step']}  K={cfg['top_k']}  seq_thresh={cfg['seq_thresh']} "
        f"({n_kept} coeffs)  thr={cfg['block_threshold']}  "
        f"gauss_sigma={cfg['gauss_sigma']}",
        sub_style))
    story.append(HRFlowable(width="100%", thickness=1,
                             color=colors.HexColor('#1a1a2e')))
    story.append(Spacer(1, 0.15*inch))

    # 1. Validation
    story.append(Paragraph("1. WHT Implementation Validation", h1_style))
    tdata = [['N', 'Cycles', 'Expected', 'Efficiency', 'Status']]
    for r in val_results:
        eff = f"{100*r['cycles']/r['expected']:.1f}%" if r['expected'] else 'N/A'
        tdata.append([str(r['N']), str(r['cycles']), str(r['expected']),
                      eff, 'PASS' if r['passed'] else 'FAIL'])
    t  = Table(tdata, colWidths=[0.8*inch]*5)
    ts = tbl()
    for i, r in enumerate(val_results, 1):
        c = colors.HexColor('#1a7a3a' if r['passed'] else '#cc0000')
        ts.add('TEXTCOLOR', (4,i),(4,i), c)
        ts.add('FONTNAME',  (4,i),(4,i), 'Helvetica-Bold')
    t.setStyle(ts); story.append(t)

    # 2. Pipeline
    story.append(Paragraph("2. Compression Pipeline", h1_style))
    story.append(Paragraph(
        "Full Walsh-domain pipeline. Transmitter: WHT per 8x8 block, "
        "sequency mask zeros coefficients where r+c > seq_thresh (Walsh-domain LPF), "
        "quantize by Q step, transmit top-K nonzero coefficients per changed block. "
        "Receiver: dequantize, IWHT back to spatial diff, Gaussian smooth to hide "
        "block boundary artifacts, add to reference frame A.", body_style))
    story.append(Spacer(1, 6))
    pdata = [['Stage', 'Parameter', 'Value', 'Notes']]
    pdata += [
        ['TX: LPF',    'seq_thresh', str(cfg['seq_thresh']),
         f'Keep coeffs r+c <= {cfg["seq_thresh"]} ({n_kept}/64) — Walsh-domain LPF'],
        ['TX: Quant',  'Q step',     str(cfg['q_step']),
         'Divide coeff, round to signed byte — kills near-zero coeffs'],
        ['TX: Select', 'top_k',      str(cfg['top_k']),
         f'Transmit up to {cfg["top_k"]} nonzero coeffs/block, sorted by magnitude'],
        ['TX: Gate',   'threshold',  str(cfg['block_threshold']),
         'Skip blocks where max masked AC < threshold'],
        ['RX: IWHT',   'block size', '8x8',
         'Inverse butterfly — 12 cycles, reconstructs spatial diff patch'],
        ['RX: Smooth', 'gauss sigma', str(cfg['gauss_sigma']),
         'Gaussian smooth over full diff image — removes 8x8 grid artifacts'],
    ]
    t = Table(pdata, colWidths=[0.9*inch, 0.9*inch, 0.7*inch, 4.2*inch])
    t.setStyle(tbl()); story.append(t)

    # 3. Noise floor
    story.append(Paragraph("3. Scene Noise Floor", h1_style))
    ndata = [['p25', 'p50', 'p75', 'p95', 'Max', 'Threshold']]
    ndata.append([f"{noise_stats['p25']:.0f}", f"{noise_stats['p50']:.0f}",
                  f"{noise_stats['p75']:.0f}", f"{noise_stats['p95']:.0f}",
                  f"{noise_stats['max']:.0f}", str(cfg['block_threshold'])])
    t  = Table(ndata, colWidths=[0.9*inch]*6)
    ts = tbl()
    tc = colors.HexColor('#1a7a3a' if cfg['block_threshold'] > noise_stats['p95']
                         else '#cc6600')
    ts.add('TEXTCOLOR', (5,1),(5,1), tc)
    ts.add('FONTNAME',  (5,1),(5,1), 'Helvetica-Bold')
    t.setStyle(ts); story.append(t)

    # 4. Per-pair results
    story.append(Paragraph("4. Per-Pair Compression Results", h1_style))
    cdata = [['Frame Pair', 'Changed', '%',
              'Bytes', 'Ratio', 'SF7 FPS', 'PSNR']]
    for r in seq_results:
        psnr_s = f"{r['psnr']:.1f}" if r['psnr'] != float('inf') else 'inf'
        cdata.append([
            r['pair'],
            str(r['changed_blocks']),
            f"{r['pct_changed']:.1f}%",
            str(r['compressed_bytes']),
            f"{r['ratio']:.1f}x",
            f"{r['sf7_fps']:.2f}",
            f"{psnr_s}dB",
        ])
    col_w = [2.0*inch, 0.6*inch, 0.5*inch, 0.65*inch, 0.7*inch, 0.65*inch, 0.75*inch]
    t  = Table(cdata, colWidths=col_w)
    ts = tbl()
    for i, r in enumerate(seq_results, 1):
        if r['ratio'] >= 220:
            ts.add('BACKGROUND', (3,i),(3,i), colors.HexColor('#d4edda'))
    t.setStyle(ts); story.append(t)

    # 5. Charts
    story.append(PageBreak())
    story.append(Paragraph("5. Compression Metrics", h1_style))

    pairs   = [r['pair'].split('->>')[0].replace('img_','').replace('.raw','')
               for r in seq_results]
    bytes_v = [r['compressed_bytes'] for r in seq_results]
    ratios  = [r['ratio']            for r in seq_results]
    fps_v   = [r['sf7_fps']          for r in seq_results]
    psnr_v  = [r['psnr'] if r['psnr'] != float('inf') else 0 for r in seq_results]
    x       = range(len(pairs))

    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))
    fig.patch.set_facecolor('#fafafa')

    ax = axes[0,0]
    ax.bar(x, bytes_v, color='#3a5fa8', alpha=0.85)
    ax.axhline(683,   color='red',    linestyle='--', lw=1, label='683B = 1 FPS')
    ax.axhline(349,   color='orange', linestyle='--', lw=1, label='349B = 220:1')
    ax.set_title('Compressed Bytes', fontweight='bold')
    ax.set_ylabel('bytes')
    ax.set_xticks(x); ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)

    ax = axes[0,1]
    bar_c = ['#1a7a3a' if r >= 220 else '#cc6600' if r >= 50 else '#cc0000'
              for r in ratios]
    ax.bar(x, ratios, color=bar_c)
    ax.axhline(220, color='orange', linestyle='--', lw=1, label='220:1 target')
    ax.set_title('Compression Ratio', fontweight='bold')
    ax.set_ylabel('x')
    ax.set_xticks(x); ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)

    ax = axes[1,0]
    fps_c = ['#1a7a3a' if f >= 1 else '#cc6600' for f in fps_v]
    ax.bar(x, fps_v, color=fps_c)
    ax.axhline(1, color='blue', linestyle='--', lw=1, label='1 FPS')
    ax.set_title('SF7 FPS', fontweight='bold')
    ax.set_ylabel('FPS')
    ax.set_xticks(x); ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)

    ax = axes[1,1]
    ax.bar(x, psnr_v, color='#7a3a8a', alpha=0.85)
    ax.axhline(30, color='green', linestyle='--', lw=1, label='30dB perceptual threshold')
    ax.set_title('Reconstruction PSNR (vs ground truth frame B)', fontweight='bold')
    ax.set_ylabel('dB')
    ax.set_xticks(x); ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)

    plt.tight_layout()
    chart_path = '/tmp/wht_charts.png'
    plt.savefig(chart_path, dpi=130, bbox_inches='tight', facecolor='#fafafa')
    plt.close()
    story.append(Image(chart_path, width=6.8*inch, height=4.8*inch))

    # 6. Summary
    story.append(Spacer(1, 0.15*inch))
    story.append(Paragraph("6. Summary Statistics", h1_style))
    all_b  = [r['compressed_bytes'] for r in seq_results]
    mean_b = sum(all_b) / len(all_b)
    hits   = sum(1 for r in seq_results if r['ratio'] >= 220)
    psnrs  = [r['psnr'] for r in seq_results if r['psnr'] != float('inf')]
    sdata  = [['Metric', 'Value']]
    sdata += [
        ['Total pairs',      str(len(seq_results))],
        ['Pairs >= 220:1',   f"{hits} ({100*hits//len(seq_results)}%)"],
        ['Mean bytes',       f"{mean_b:.0f}"],
        ['Mean ratio',       f"{76800/mean_b:.1f}x"],
        ['Mean SF7 FPS',     f"{683/mean_b:.2f}"],
        ['Mean PSNR',        f"{sum(psnrs)/len(psnrs):.1f} dB" if psnrs else 'N/A'],
        ['Best ratio',       f"{max(r['ratio'] for r in seq_results):.0f}x"],
        ['Worst ratio',      f"{min(r['ratio'] for r in seq_results):.1f}x"],
    ]
    t = Table(sdata, colWidths=[2.2*inch, 4.1*inch])
    t.setStyle(tbl()); story.append(t)

    story.append(Spacer(1, 0.15*inch))
    story.append(HRFlowable(width="100%", thickness=0.5,
                             color=colors.HexColor('#cccccc')))
    story.append(Spacer(1, 6))
    story.append(Paragraph(
        "SD403 Trail Camera IV — NDSU Senior Design | SDD501 WHT Compression | "
        f"Generated {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        ParagraphStyle('Footer', parent=styles['Normal'],
                       fontSize=7, textColor=colors.grey, alignment=TA_CENTER)))
    doc.build(story)
    print(f"  Report saved: {output_path}")


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='WHT Simulation + Reconstruction Visualizer')
    parser.add_argument('--report',     action='store_true')
    parser.add_argument('--visualize',  action='store_true',
                        help='Output reconstruction PNG for first motion pair')
    parser.add_argument('--frames',     default=None)
    parser.add_argument('--frame-a',    default=None)
    parser.add_argument('--frame-b',    default=None)
    parser.add_argument('--threshold',  type=int,   default=400)
    parser.add_argument('--q-step',     type=int,   default=32)
    parser.add_argument('--top-k',      type=int,   default=5,
                        help='Max coefficients per block patch (default: 5)')
    parser.add_argument('--seq-thresh', type=int,   default=2,
                        help='Sequency LPF threshold (default: 2 = 6 coeffs)')
    parser.add_argument('--gauss-sigma',type=float, default=4.0,
                        help='Receiver Gaussian smooth sigma (default: 4.0)')
    parser.add_argument('--row',        type=int,   default=100)
    parser.add_argument('--col',        type=int,   default=160)
    parser.add_argument('--out',        default='wht_report.pdf')
    args = parser.parse_args()

    cfg = {
        'q_step':          args.q_step,
        'top_k':           args.top_k,
        'seq_thresh':      args.seq_thresh,
        'block_threshold': args.threshold,
        'gauss_sigma':     args.gauss_sigma,
    }

    # Validation
    print("Walsh-Hadamard Transform Validation")
    print("=" * 60)
    val_results = []
    for exp in range(1, 8):
        N = 1 << exp
        r = validate(N)
        val_results.append(r)
        print(f"N={N:4d} | cycles={r['cycles']:5d} | expected={r['expected']:5d} | PASS")
    print("=" * 60)

    # Frame loading
    frame_dir = args.frames
    if frame_dir is None:
        for c in ['../ESP_Ecosystem/esp32cam/docs/testing/artifact/test3',
                  '../ESP_Ecosystem/esp32cam/docs/testing/artifact/test4',
                  './frames', '.']:
            if glob.glob(os.path.join(c, 'img_*.raw')):
                frame_dir = c
                break

    frames = []
    if frame_dir and os.path.isdir(frame_dir):
        frames = load_frame_dir(frame_dir)
        print(f"\nLoaded {len(frames)} frames from {frame_dir}")
    elif frame_dir:
        print(f"\nWarning: '{frame_dir}' not found")

    # Demo block WHT
    fa_path = args.frame_a
    fb_path = args.frame_b
    if fa_path is None and len(frames) >= 2:
        fa_path = os.path.join(frame_dir, frames[0][0])
        fb_path = os.path.join(frame_dir, frames[1][0])

    if fa_path and fb_path and os.path.exists(fa_path) and os.path.exists(fb_path):
        frame_a  = load_frame(fa_path)
        frame_b  = load_frame(fb_path)
        raw_diff = diff_frames(frame_a, frame_b)
        block    = [raw_diff[y][args.col:args.col+8]
                    for y in range(args.row, args.row+8)]
        coeffs   = wht_2D(block)
        masked   = sequency_mask(coeffs, cfg['seq_thresh'])

        print(f"\nDemo 8x8 block  row={args.row} col={args.col}")
        print(f"{'Row':<4} {'--- diff (pre-mask) ---':<50} {'--- WHT coefficients ---'}")
        print("-" * 100)
        for r in range(8):
            px = [int(block[r][c]) for c in range(8)]
            co = coeffs[r]
            print(f"{r:<4} {'  '.join(f'{v:4d}' for v in px)}    "
                  f"{'  '.join(f'{v:6d}' for v in co)}")

        n_kept  = sum(1 for r in range(8) for c in range(8) if r+c <= cfg['seq_thresh'])
        total_e = sum(coeffs[r][c]**2 for r in range(8) for c in range(8))
        kept_e  = sum(masked[r][c]**2 for r in range(8) for c in range(8))
        print(f"\n  seq_thresh={cfg['seq_thresh']}: keeping {n_kept}/64 coefficients")
        print(f"  Energy retained after mask: "
              f"{100*kept_e/total_e:.1f}%" if total_e else "  N/A")

        print(f"\n--- Round-trip  Q={cfg['q_step']} K={cfg['top_k']} "
              f"seq_thresh={cfg['seq_thresh']} sigma={cfg['gauss_sigma']} ---")
        r = encode_decode(frame_a, frame_b, **cfg)
        print(f"  Changed blocks:  {r['changed_blocks']} ({r['pct_changed']:.1f}%)")
        print(f"  Compressed:      {r['compressed_bytes']} bytes")
        print(f"  Ratio:           {r['ratio']:.1f}x")
        print(f"  SF7 FPS:         {r['sf7_fps']:.2f}")
        print(f"  PSNR:            {r['psnr']:.1f} dB")

        if args.visualize:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import numpy as np
            fa_np = np.array([[float(frame_a[y][x]) for x in range(320)]
                               for y in range(240)])
            fig, axes = plt.subplots(1, 3, figsize=(15, 5), facecolor='#0d0d0d')
            for ax, (title, img) in zip(axes, [
                ('Frame A (reference)',        fa_np),
                ('Frame B (ground truth)',     r['fb_np']),
                (f"Reconstructed\n{r['psnr']:.1f} dB  {r['compressed_bytes']}B  "
                 f"{r['ratio']:.0f}x", r['recon']),
            ]):
                ax.imshow(img, cmap='gray', vmin=0, vmax=255, interpolation='nearest')
                ax.set_title(title, color='white', fontsize=9, pad=5)
                ax.axis('off')
            fig.suptitle(
                f"Q={cfg['q_step']}  K={cfg['top_k']}  seq_thresh={cfg['seq_thresh']}"
                f"  sigma={cfg['gauss_sigma']}  {r['changed_blocks']} blocks",
                color='white', fontsize=9)
            plt.tight_layout()
            vpath = args.out.replace('.pdf', '_recon.png')
            plt.savefig(vpath, dpi=150, bbox_inches='tight', facecolor='#0d0d0d')
            plt.close()
            print(f"  Visualization saved: {vpath}")

    # Sequence analysis
    if len(frames) >= 2:
        print(f"\n--- Sequence Analysis  {len(frames)} frames / {len(frames)-1} pairs ---")
        print(f"    Q={cfg['q_step']}  K={cfg['top_k']}  "
              f"seq_thresh={cfg['seq_thresh']}  thr={cfg['block_threshold']}  "
              f"sigma={cfg['gauss_sigma']}\n")

        seq_results = analyze_frame_sequence(frames, **cfg)

        print(f"{'Pair':<35} {'Bytes':>7} {'Ratio':>8} {'FPS':>6} {'PSNR':>8} {'Changed':>8}")
        print("-" * 78)
        for r in seq_results:
            psnr_s = f"{r['psnr']:.1f}dB" if r['psnr'] != float('inf') else "   inf"
            print(f"{r['pair']:<35} {r['compressed_bytes']:>7} "
                  f"{r['ratio']:>8.1f}x {r['sf7_fps']:>6.2f} "
                  f"{psnr_s:>8} {r['changed_blocks']:>8}")

        all_b  = [r['compressed_bytes'] for r in seq_results]
        mean_b = sum(all_b) / len(all_b)
        hits   = sum(1 for r in seq_results if r['ratio'] >= 220)
        print(f"\n  mean={mean_b:.0f}B  mean_ratio={76800/mean_b:.1f}x  "
              f">=220:1: {hits}/{len(seq_results)}  "
              f"min={min(all_b)}B  max={max(all_b)}B")

        nf = noise_floor_stats(frames)
        print(f"\n  Noise floor  p25={nf['p25']:.0f}  p50={nf['p50']:.0f}  "
              f"p75={nf['p75']:.0f}  p95={nf['p95']:.0f}")
        print(f"  Threshold {cfg['block_threshold']} is "
              f"{'above' if cfg['block_threshold'] > nf['p95'] else 'BELOW'} p95")

        if args.report:
            print(f"\nGenerating PDF -> {args.out}")
            generate_report(val_results, seq_results, nf,
                            frame_dir or '.', cfg, args.out)
    else:
        if args.report:
            print("Need >= 2 frames. Use --frames <dir>")

    if not args.report and len(frames) >= 2:
        print("\nTip: --report to generate PDF  |  --visualize to output reconstruction PNG")


if __name__ == "__main__":
    main()
