"""
swp_wht.py -- Parameter sweep for WHT compression pipeline

Mirrors swp_val.m structure: sweep parameters, plot rate-distortion curves.

Usage:
    python3 swp_wht.py --frames <dir>
    python3 swp_wht.py --frames ../esp/esp32cam/docs/testing/artifact/test3

Sweeps:
    1. q_step sweep       -- Rate-distortion curve (PSNR vs bytes)
    2. top_k sweep        -- Coefficient budget impact
    3. threshold sweep    -- Noise gate sensitivity
    4. Combined RD curve  -- q_step x top_k grid

Outputs to plt/<dataset>/swp/:
    swp_qstep.png         -- PSNR and bytes vs q_step
    swp_topk.png          -- PSNR and bytes vs top_k
    swp_threshold.png     -- PSNR and bytes vs block_threshold
    swp_rd_curve.png      -- Rate-distortion: PSNR vs bytes (all configs)
"""

import os
import sys
import argparse
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from frames import load_frame_dir
from codec import encode_decode, noise_floor_stats


# --------------------------------------------------------
#  Specifications
# --------------------------------------------------------
Q_STEP          = 32
TOP_K           = 5
SEQ_THRESH      = 2
BLOCK_THRESHOLD = 400
GAUSS_SIGMA     = 4.0


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def mean_metrics(frames, **cfg):
    """Run encode_decode on all consecutive pairs, return mean metrics."""
    psnrs  = []
    bytess = []
    ratios = []
    for i in range(len(frames) - 1):
        _, fa = frames[i]
        _, fb = frames[i + 1]
        r = encode_decode(fa, fb, **cfg)
        psnrs.append(r['psnr'] if r['psnr'] != float('inf') else 80.0)
        bytess.append(r['compressed_bytes'])
        ratios.append(r['ratio'])
    return {
        'mean_psnr':  np.mean(psnrs),
        'mean_bytes': np.mean(bytess),
        'mean_ratio': np.mean(ratios),
        'all_psnr':   psnrs,
        'all_bytes':  bytess,
    }


# --------------------------------------------------------
#  Sweep 1: q_step
# --------------------------------------------------------
def swp_qstep(frames, dest, base_cfg):
    q_values = [4, 8, 16, 24, 32, 48, 64, 96, 128]

    results = []
    print("\n--- q_step Sweep ---")
    print(f"  {'q_step':>6} {'PSNR':>8} {'Bytes':>8} {'Ratio':>8}")
    print(f"  {'-'*34}")

    for q in q_values:
        cfg = {**base_cfg, 'q_step': q}
        m = mean_metrics(frames, **cfg)
        results.append((q, m))
        print(f"  {q:>6} {m['mean_psnr']:>8.1f} {m['mean_bytes']:>8.0f} "
              f"{m['mean_ratio']:>8.1f}")

    # Plot: dual y-axis -- PSNR (left) and bytes (right) vs q_step
    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax2 = ax1.twinx()

    qs    = [r[0] for r in results]
    psnrs = [r[1]['mean_psnr'] for r in results]
    bytss = [r[1]['mean_bytes'] for r in results]

    l1, = ax1.plot(qs, psnrs, 'bo-', linewidth=2, markersize=8, label='PSNR (dB)')
    l2, = ax2.plot(qs, bytss, 'rs--', linewidth=2, markersize=8, label='Bytes')

    # Mark the current operating point
    cur_q = base_cfg['q_step']
    cur_idx = qs.index(cur_q) if cur_q in qs else None
    if cur_idx is not None:
        ax1.axvline(cur_q, color='gray', linestyle=':', alpha=0.5)
        ax1.annotate(f'current Q={cur_q}', xy=(cur_q, psnrs[cur_idx]),
                     xytext=(cur_q + 5, psnrs[cur_idx] + 1),
                     fontsize=9, color='gray',
                     arrowprops=dict(arrowstyle='->', color='gray'))

    # 6 dB/bit reference line
    ax1.axhline(30, color='green', linestyle='--', alpha=0.5, linewidth=1)
    ax1.text(qs[-1], 30.5, '30dB', fontsize=8, color='green')

    # SF7 1 FPS line
    ax2.axhline(683, color='red', linestyle='--', alpha=0.5, linewidth=1)
    ax2.text(qs[0], 700, '683B (1 FPS)', fontsize=8, color='red')

    ax1.set_xlabel('q_step', fontsize=12)
    ax1.set_ylabel('Mean PSNR (dB)', color='blue', fontsize=12)
    ax2.set_ylabel('Mean Compressed Bytes', color='red', fontsize=12)
    ax1.set_title(f'q_step Sweep -- K={base_cfg["top_k"]}  '
                  f'seq={base_cfg["seq_thresh"]}  thr={base_cfg["block_threshold"]}',
                  fontweight='bold')
    ax1.legend(handles=[l1, l2], loc='center right', fontsize=9)
    ax1.grid(True, alpha=0.3)

    fname = os.path.join(dest, 'swp_qstep.png')
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved {fname}")
    return results


# --------------------------------------------------------
#  Sweep 2: top_k
# --------------------------------------------------------
def swp_topk(frames, dest, base_cfg):
    # Max coefficients with seq_thresh=2 is 6
    n_kept = sum(1 for r in range(8) for c in range(8)
                 if r + c <= base_cfg['seq_thresh'])
    k_values = list(range(1, n_kept + 1))

    results = []
    print("\n--- top_k Sweep ---")
    print(f"  {'top_k':>6} {'PSNR':>8} {'Bytes':>8} {'Ratio':>8}")
    print(f"  {'-'*34}")

    for k in k_values:
        cfg = {**base_cfg, 'top_k': k}
        m = mean_metrics(frames, **cfg)
        results.append((k, m))
        print(f"  {k:>6} {m['mean_psnr']:>8.1f} {m['mean_bytes']:>8.0f} "
              f"{m['mean_ratio']:>8.1f}")

    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax2 = ax1.twinx()

    ks    = [r[0] for r in results]
    psnrs = [r[1]['mean_psnr'] for r in results]
    bytss = [r[1]['mean_bytes'] for r in results]

    ax1.bar(np.array(ks) - 0.15, psnrs, width=0.3, color='#3a5fa8',
            alpha=0.85, label='PSNR (dB)')
    ax2.bar(np.array(ks) + 0.15, bytss, width=0.3, color='#cc4444',
            alpha=0.85, label='Bytes')

    cur_k = base_cfg['top_k']
    if cur_k in ks:
        ax1.axvline(cur_k, color='gray', linestyle=':', alpha=0.5)

    ax1.set_xlabel('top_k (coefficients per block)', fontsize=12)
    ax1.set_ylabel('Mean PSNR (dB)', color='blue', fontsize=12)
    ax2.set_ylabel('Mean Compressed Bytes', color='red', fontsize=12)
    ax1.set_title(f'top_k Sweep -- Q={base_cfg["q_step"]}  '
                  f'seq={base_cfg["seq_thresh"]}  thr={base_cfg["block_threshold"]}',
                  fontweight='bold')
    ax1.set_xticks(ks)
    ax1.grid(True, alpha=0.3, axis='y')
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='center right', fontsize=9)

    fname = os.path.join(dest, 'swp_topk.png')
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved {fname}")
    return results


# --------------------------------------------------------
#  Sweep 3: block_threshold
# --------------------------------------------------------
def swp_threshold(frames, dest, base_cfg):
    thr_values = [100, 200, 400, 600, 800, 1000, 1200, 1500, 2000, 3000]

    # Get noise floor for reference
    nf = noise_floor_stats(frames)

    results = []
    print("\n--- threshold Sweep ---")
    print(f"  Noise floor p95 = {nf['p95']:.0f}")
    print(f"  {'thr':>6} {'PSNR':>8} {'Bytes':>8} {'Ratio':>8}")
    print(f"  {'-'*34}")

    for thr in thr_values:
        cfg = {**base_cfg, 'block_threshold': thr}
        m = mean_metrics(frames, **cfg)
        results.append((thr, m))
        marker = " <-- p95" if abs(thr - nf['p95']) < 200 else ""
        print(f"  {thr:>6} {m['mean_psnr']:>8.1f} {m['mean_bytes']:>8.0f} "
              f"{m['mean_ratio']:>8.1f}{marker}")

    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax2 = ax1.twinx()

    thrs  = [r[0] for r in results]
    psnrs = [r[1]['mean_psnr'] for r in results]
    bytss = [r[1]['mean_bytes'] for r in results]

    l1, = ax1.plot(thrs, psnrs, 'bo-', linewidth=2, markersize=8, label='PSNR (dB)')
    l2, = ax2.plot(thrs, bytss, 'rs--', linewidth=2, markersize=8, label='Bytes')

    # Noise floor reference
    ax1.axvline(nf['p95'], color='orange', linestyle='--', linewidth=2,
                label=f"Noise floor p95 = {nf['p95']:.0f}")
    ax1.axvspan(0, nf['p95'], alpha=0.08, color='red')
    ax1.text(nf['p95'] * 0.5, max(psnrs) - 0.5, 'NOISE\nZONE',
             fontsize=10, color='red', alpha=0.5, ha='center', fontweight='bold')

    ax1.set_xlabel('block_threshold', fontsize=12)
    ax1.set_ylabel('Mean PSNR (dB)', color='blue', fontsize=12)
    ax2.set_ylabel('Mean Compressed Bytes', color='red', fontsize=12)
    ax1.set_title(f'Threshold Sweep -- Q={base_cfg["q_step"]}  K={base_cfg["top_k"]}',
                  fontweight='bold')
    ax1.legend(handles=[l1, l2], loc='center right', fontsize=9)
    ax1.grid(True, alpha=0.3)

    fname = os.path.join(dest, 'swp_threshold.png')
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved {fname}")
    return results, nf


# --------------------------------------------------------
#  Sweep 4: Rate-distortion curve (q_step x top_k grid)
# --------------------------------------------------------
def swp_rd_curve(frames, dest, base_cfg):
    """Combined sweep: plot PSNR vs bytes for multiple (q_step, top_k) combos."""
    q_values = [8, 16, 32, 64, 128]
    k_values = [1, 3, 5, 6]

    print("\n--- Rate-Distortion Grid ---")
    print(f"  q_step: {q_values}")
    print(f"  top_k:  {k_values}")

    fig, ax = plt.subplots(figsize=(10, 7))
    colors = ['#1a7a3a', '#3a5fa8', '#cc6600', '#cc0000']
    markers = ['o', 's', '^', 'D']

    for ki, k in enumerate(k_values):
        psnrs = []
        bytss = []
        for q in q_values:
            cfg = {**base_cfg, 'q_step': q, 'top_k': k}
            m = mean_metrics(frames, **cfg)
            psnrs.append(m['mean_psnr'])
            bytss.append(m['mean_bytes'])
            print(f"  Q={q:>3}  K={k}  -> {m['mean_psnr']:.1f}dB  {m['mean_bytes']:.0f}B")

        ax.plot(bytss, psnrs, f'-{markers[ki]}', color=colors[ki],
                linewidth=2, markersize=8, label=f'K={k}')

        # Annotate q_step values on the K=5 line
        if k == base_cfg['top_k']:
            for qi, q in enumerate(q_values):
                ax.annotate(f'Q={q}', xy=(bytss[qi], psnrs[qi]),
                            xytext=(5, 5), textcoords='offset points',
                            fontsize=7, color=colors[ki])

    # Reference lines
    ax.axhline(30, color='green', linestyle='--', alpha=0.5, linewidth=1)
    ax.text(ax.get_xlim()[1] * 0.95, 30.5, '30dB', fontsize=8,
            color='green', ha='right')
    ax.axvline(683, color='red', linestyle='--', alpha=0.5, linewidth=1)
    ax.text(700, ax.get_ylim()[1] * 0.95, '1 FPS', fontsize=8,
            color='red', rotation=90, va='top')

    ax.set_xlabel('Mean Compressed Bytes', fontsize=12)
    ax.set_ylabel('Mean PSNR (dB)', fontsize=12)
    ax.set_title('Rate-Distortion Curve', fontweight='bold', fontsize=14)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    fname = os.path.join(dest, 'swp_rd_curve.png')
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved {fname}")


# --------------------------------------------------------
#  Main
# --------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='WHT Parameter Sweep (like swp_val.m)')
    parser.add_argument('--frames',     required=True, help='Frame directory')
    parser.add_argument('--threshold',  type=int,   default=BLOCK_THRESHOLD)
    parser.add_argument('--q-step',     type=int,   default=Q_STEP)
    parser.add_argument('--top-k',      type=int,   default=TOP_K)
    parser.add_argument('--seq-thresh', type=int,   default=SEQ_THRESH)
    parser.add_argument('--gauss-sigma',type=float, default=GAUSS_SIGMA)
    parser.add_argument('--sweep',      default='all',
                        choices=['all', 'qstep', 'topk', 'threshold', 'rd'],
                        help='Which sweep to run (default: all)')
    args = parser.parse_args()

    base_cfg = {
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

    dataset = os.path.basename(os.path.normpath(args.frames)) or 'default'
    dest = os.path.join('plt', dataset, 'swp')
    ensure_dir(dest)

    print(f"\nswp_wht -- Parameter sweeps -> {dest}/")
    print(f"  {len(frames)} frames, {len(frames)-1} pairs")
    print(f"  Base config: Q={base_cfg['q_step']}  K={base_cfg['top_k']}  "
          f"seq={base_cfg['seq_thresh']}  thr={base_cfg['block_threshold']}")

    if args.sweep in ('all', 'qstep'):
        swp_qstep(frames, dest, base_cfg)
    if args.sweep in ('all', 'topk'):
        swp_topk(frames, dest, base_cfg)
    if args.sweep in ('all', 'threshold'):
        swp_threshold(frames, dest, base_cfg)
    if args.sweep in ('all', 'rd'):
        swp_rd_curve(frames, dest, base_cfg)

    print(f"\nDone. Plots in {dest}/")


if __name__ == "__main__":
    main()
