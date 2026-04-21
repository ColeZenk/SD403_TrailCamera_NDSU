"""
plt_wht.py -- Generate WHT compression visualization plots

Mirrors plt_bsf.m structure: load results, generate figures, save to plt/.

Usage:
    python3 plt_wht.py --frames <dir>
    python3 plt_wht.py --frames ../esp/esp32cam/docs/testing/artifact/test3

Outputs to plt/<dataset_name>/:
    recon_pair_NN.png      -- Frame A | Ground Truth B | Reconstructed B
    noise_floor.png        -- Noise floor histogram + threshold line
    per_pair_metrics.png   -- Bytes, ratio, PSNR, FPS per pair (4-panel)
    block_activity.png     -- Heatmap of changed blocks per pair
"""

import os
import sys
import argparse
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from frames import load_frame, load_frame_dir
from codec import (diff_frames, noise_floor_stats, encode_frame,
                   decode_frame, encode_decode, sequency_mask)
from wht_core import wht_2D


# --------------------------------------------------------
#  Specifications (match wht_sim.py defaults)
# --------------------------------------------------------
Q_STEP          = 32
TOP_K           = 5
SEQ_THRESH      = 2
BLOCK_THRESHOLD = 400
GAUSS_SIGMA     = 4.0

PIXELS_X = 320
PIXELS_Y = 240
FRAME_BYTES = PIXELS_X * PIXELS_Y   # 76800


def frame_to_np(frame):
    return np.array([[float(frame[y][x]) for x in range(PIXELS_X)]
                      for y in range(PIXELS_Y)])


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


# --------------------------------------------------------
#  Figure 1: Reconstruction comparison (per pair)
# --------------------------------------------------------
def plt_recon(frame_a, frame_b, cfg, pair_name, dest):
    """Frame A | Ground Truth B | Reconstructed B -- side by side."""
    r = encode_decode(frame_a, frame_b, **cfg)

    fa_np = frame_to_np(frame_a)
    fb_np = r['fb_np']
    recon = r['recon']

    fig, axes = plt.subplots(1, 3, figsize=(15, 5), facecolor='#0d0d0d')

    for ax, (title, img) in zip(axes, [
        ('Frame A (reference)', fa_np),
        ('Frame B (ground truth)', fb_np),
        (f"Reconstructed B\n"
         f"PSNR={r['psnr']:.1f}dB  {r['compressed_bytes']}B  "
         f"{r['ratio']:.0f}:1", recon),
    ]):
        ax.imshow(img, cmap='gray', vmin=0, vmax=255, interpolation='nearest')
        ax.set_title(title, color='white', fontsize=10, pad=6)
        ax.axis('off')

    fig.suptitle(
        f"{pair_name}    Q={cfg['q_step']}  K={cfg['top_k']}  "
        f"seq={cfg['seq_thresh']}  thr={cfg['block_threshold']}  "
        f"sigma={cfg['gauss_sigma']}",
        color='white', fontsize=10, y=0.02)
    plt.tight_layout()

    fname = os.path.join(dest, f"recon_{pair_name.replace('>>','_')}.png")
    plt.savefig(fname, dpi=150, bbox_inches='tight', facecolor='#0d0d0d')
    plt.close()
    return fname, r


# --------------------------------------------------------
#  Figure 2: Noise floor histogram
# --------------------------------------------------------
def plt_noise_floor(frames, cfg, dest):
    """Histogram of max AC WHT coefficient per block across all pairs."""
    all_ac_max = []
    for fi in range(len(frames) - 1):
        _, fa = frames[fi]
        _, fb = frames[fi + 1]
        diff = diff_frames(fa, fb)
        for by in range(0, PIXELS_Y, 8):
            for bx in range(0, PIXELS_X, 8):
                block  = [diff[y][bx:bx+8] for y in range(by, by+8)]
                coeffs = wht_2D(block)
                ac = [abs(coeffs[r][c])
                      for r in range(8) for c in range(8)
                      if not (r == 0 and c == 0)]
                all_ac_max.append(max(ac))

    all_ac_max = np.array(all_ac_max)
    p95 = np.percentile(all_ac_max, 95)
    p50 = np.percentile(all_ac_max, 50)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.hist(all_ac_max, bins=100, color='#3a5fa8', alpha=0.8, edgecolor='none')
    ax.axvline(cfg['block_threshold'], color='red', linestyle='--', linewidth=2,
               label=f"block_threshold = {cfg['block_threshold']}")
    ax.axvline(p95, color='orange', linestyle='--', linewidth=1.5,
               label=f"p95 = {p95:.0f}")
    ax.axvline(p50, color='green', linestyle='--', linewidth=1.5,
               label=f"p50 = {p50:.0f}")

    # Shade the region below threshold that gets transmitted (noise)
    if cfg['block_threshold'] < p95:
        ax.axvspan(cfg['block_threshold'], p95,
                   alpha=0.15, color='red', label='Noise transmitted')

    ax.set_xlabel('max |AC coefficient| per 8x8 block')
    ax.set_ylabel('Count (blocks)')
    ax.set_title('Noise Floor Distribution -- max AC per block across all pairs')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    fname = os.path.join(dest, 'noise_floor.png')
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    return fname


# --------------------------------------------------------
#  Figure 3: Per-pair metrics (4-panel, like report.py)
# --------------------------------------------------------
def plt_per_pair(seq_results, cfg, dest):
    """Bytes, ratio, PSNR, FPS -- bar charts per frame pair."""
    pairs   = [r['pair'].split('->>')[0].replace('img_','').replace('.raw','')
               for r in seq_results]
    bytes_v = [r['compressed_bytes'] for r in seq_results]
    ratios  = [r['ratio']            for r in seq_results]
    fps_v   = [r['sf7_fps']          for r in seq_results]
    psnr_v  = [r['psnr'] if r['psnr'] != float('inf') else 0
               for r in seq_results]
    x = np.arange(len(pairs))

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))

    # Bytes
    ax = axes[0, 0]
    ax.bar(x, bytes_v, color='#3a5fa8', alpha=0.85)
    ax.axhline(683, color='red', linestyle='--', lw=1, label='683B = 1 FPS @ SF7')
    ax.axhline(349, color='orange', linestyle='--', lw=1, label='349B = 220:1')
    ax.set_title('Compressed Bytes per Pair', fontweight='bold')
    ax.set_ylabel('bytes')
    ax.set_xticks(x)
    ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Ratio
    ax = axes[0, 1]
    bar_c = ['#1a7a3a' if r >= 220 else '#cc6600' if r >= 50 else '#cc0000'
             for r in ratios]
    ax.bar(x, ratios, color=bar_c)
    ax.axhline(220, color='orange', linestyle='--', lw=1, label='220:1 target')
    ax.set_title('Compression Ratio', fontweight='bold')
    ax.set_ylabel('ratio (x)')
    ax.set_xticks(x)
    ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # FPS
    ax = axes[1, 0]
    fps_c = ['#1a7a3a' if f >= 1 else '#cc6600' for f in fps_v]
    ax.bar(x, fps_v, color=fps_c)
    ax.axhline(1, color='blue', linestyle='--', lw=1, label='1 FPS')
    ax.set_title('SF7 Achievable FPS', fontweight='bold')
    ax.set_ylabel('FPS')
    ax.set_xticks(x)
    ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # PSNR
    ax = axes[1, 1]
    ax.bar(x, psnr_v, color='#7a3a8a', alpha=0.85)
    ax.axhline(30, color='green', linestyle='--', lw=1, label='30dB threshold')
    ax.set_title('PSNR (vs ground truth B)', fontweight='bold')
    ax.set_ylabel('dB')
    ax.set_xticks(x)
    ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    fig.suptitle(
        f"Q={cfg['q_step']}  K={cfg['top_k']}  seq={cfg['seq_thresh']}  "
        f"thr={cfg['block_threshold']}  sigma={cfg['gauss_sigma']}",
        fontsize=11, fontweight='bold')
    plt.tight_layout()

    fname = os.path.join(dest, 'per_pair_metrics.png')
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    return fname


# --------------------------------------------------------
#  Figure 4: Block activity heatmap
# --------------------------------------------------------
def plt_block_activity(frame_a, frame_b, cfg, pair_name, dest):
    """Heatmap showing which 8x8 blocks are active (transmitted)."""
    diff = diff_frames(frame_a, frame_b)
    _, encoded = encode_frame(diff, q_step=cfg['q_step'], top_k=cfg['top_k'],
                              seq_thresh=cfg['seq_thresh'],
                              block_threshold=cfg['block_threshold'])

    grid = np.zeros((PIXELS_Y // 8, PIXELS_X // 8))
    for by, bx, selected in encoded:
        grid[by // 8, bx // 8] = len(selected)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Block activity
    ax = axes[0]
    im = ax.imshow(grid, cmap='hot', interpolation='nearest', aspect='auto')
    ax.set_title(f'Block Activity -- {pair_name}\n'
                 f'({len(encoded)} blocks transmitted)', fontsize=10)
    ax.set_xlabel('Block column (0-39)')
    ax.set_ylabel('Block row (0-29)')
    plt.colorbar(im, ax=ax, label='# coefficients')

    # Overlay on frame
    ax = axes[1]
    fa_np = frame_to_np(frame_a)
    ax.imshow(fa_np, cmap='gray', vmin=0, vmax=255, interpolation='nearest')
    # Draw red rectangles on active blocks
    for by, bx, selected in encoded:
        rect = plt.Rectangle((bx, by), 8, 8, linewidth=0.5,
                              edgecolor='red', facecolor='red', alpha=0.3)
        ax.add_patch(rect)
    ax.set_title(f'Active blocks overlaid on Frame A', fontsize=10)
    ax.axis('off')

    plt.tight_layout()
    fname = os.path.join(dest, f"blocks_{pair_name.replace('>>','_')}.png")
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    return fname


# --------------------------------------------------------
#  Figure 5: Error image (spatial distribution of reconstruction error)
# --------------------------------------------------------
def plt_error_map(frame_a, frame_b, cfg, pair_name, dest):
    """Absolute error |ground_truth - reconstructed| as heatmap."""
    r = encode_decode(frame_a, frame_b, **cfg)

    error = np.abs(r['fb_np'] - r['recon'])

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))

    ax = axes[0]
    ax.imshow(r['fb_np'], cmap='gray', vmin=0, vmax=255, interpolation='nearest')
    ax.set_title('Ground Truth B', fontsize=10)
    ax.axis('off')

    ax = axes[1]
    ax.imshow(r['recon'], cmap='gray', vmin=0, vmax=255, interpolation='nearest')
    ax.set_title(f'Reconstructed B  (PSNR={r["psnr"]:.1f}dB)', fontsize=10)
    ax.axis('off')

    ax = axes[2]
    im = ax.imshow(error, cmap='hot', interpolation='nearest', vmin=0)
    ax.set_title(f'|Error|  (max={error.max():.1f}, mean={error.mean():.1f})',
                 fontsize=10)
    ax.axis('off')
    plt.colorbar(im, ax=ax, label='Absolute error (gray levels)')

    fig.suptitle(f"{pair_name}    Q={cfg['q_step']}  K={cfg['top_k']}",
                 fontsize=10, fontweight='bold')
    plt.tight_layout()

    fname = os.path.join(dest, f"error_{pair_name.replace('>>','_')}.png")
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    return fname


# --------------------------------------------------------
#  Main -- generate all plots
# --------------------------------------------------------
def generate_plots(frames, frame_dir, cfg, max_recon_pairs=5):
    """Generate all plots to plt/<dataset>/."""
    dataset = os.path.basename(os.path.normpath(frame_dir)) or 'default'
    dest = os.path.join('plt', dataset)
    ensure_dir(dest)

    print(f"\n{'='*60}")
    print(f"  plt_wht -- Generating plots to {dest}/")
    print(f"  {len(frames)} frames, {len(frames)-1} pairs")
    print(f"  Q={cfg['q_step']}  K={cfg['top_k']}  seq={cfg['seq_thresh']}  "
          f"thr={cfg['block_threshold']}  sigma={cfg['gauss_sigma']}")
    print(f"{'='*60}\n")

    # Noise floor
    print("  [1/4] Noise floor histogram...", end=' ', flush=True)
    f = plt_noise_floor(frames, cfg, dest)
    print(f"Saved {f}")

    # Per-pair metrics
    print("  [2/4] Per-pair metrics...", end=' ', flush=True)
    from codec import analyze_frame_sequence
    seq_results = analyze_frame_sequence(frames, **cfg)
    f = plt_per_pair(seq_results, cfg, dest)
    print(f"Saved {f}")

    # Reconstruction + error + block activity for first N pairs
    n = min(max_recon_pairs, len(frames) - 1)
    for i in range(n):
        name_a, fa = frames[i]
        name_b, fb = frames[i + 1]
        pair_name = f"{name_a.replace('.raw','')}>>{name_b.replace('.raw','')}"

        print(f"  [3/4] Pair {i+1}/{n}: {pair_name}")

        f1, _ = plt_recon(fa, fb, cfg, pair_name, dest)
        print(f"         recon   -> {f1}")

        f2 = plt_error_map(fa, fb, cfg, pair_name, dest)
        print(f"         error   -> {f2}")

        f3 = plt_block_activity(fa, fb, cfg, pair_name, dest)
        print(f"         blocks  -> {f3}")

    print(f"\n  [4/4] Done. All plots in {dest}/")
    return dest


def main():
    parser = argparse.ArgumentParser(
        description='WHT Compression Plots (like plt_bsf.m)')
    parser.add_argument('--frames',     required=True, help='Frame directory')
    parser.add_argument('--threshold',  type=int,   default=BLOCK_THRESHOLD)
    parser.add_argument('--q-step',     type=int,   default=Q_STEP)
    parser.add_argument('--top-k',      type=int,   default=TOP_K)
    parser.add_argument('--seq-thresh', type=int,   default=SEQ_THRESH)
    parser.add_argument('--gauss-sigma',type=float, default=GAUSS_SIGMA)
    parser.add_argument('--max-pairs',  type=int,   default=5,
                        help='Max reconstruction pairs to plot (default: 5)')
    args = parser.parse_args()

    cfg = {
        'q_step':          args.q_step,
        'top_k':           args.top_k,
        'seq_thresh':      args.seq_thresh,
        'block_threshold': args.threshold,
        'gauss_sigma':     args.gauss_sigma,
    }

    frames = load_frame_dir(args.frames)
    if len(frames) < 2:
        print("Need >= 2 frames.")
        sys.exit(1)

    generate_plots(frames, args.frames, cfg, max_recon_pairs=args.max_pairs)


if __name__ == "__main__":
    main()
