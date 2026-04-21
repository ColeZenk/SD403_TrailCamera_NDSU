#!/usr/bin/env python3
"""
SDD501 - Walsh-Hadamard Transform Simulation
CLI entry point — imports pipeline from modules.

Run with --report to generate a PDF test report.
"""

import os
import glob
import argparse

from wht_core import validate, wht_2D, fwht_reference
from frames import load_frame, load_frame_dir
from codec import (diff_frames, sequency_mask, noise_floor_stats,
                   encode_frame, decode_frame, encode_decode,
                   analyze_frame_sequence)
from motion import (encode_decode_mc, analyze_frame_sequence_mc)
from report import generate_report
from plt_wht import generate_plots


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
    parser.add_argument('--gauss-sigma',  type=float, default=4.0,
                        help='Receiver Gaussian smooth sigma (default: 4.0)')
    parser.add_argument('--plot',          action='store_true',
                        help='Generate all visualization plots to plt/')
    parser.add_argument('--sweep',         action='store_true',
                        help='Run parameter sweeps to plt/<dataset>/swp/')
    parser.add_argument('--mc',            action='store_true',
                        help='Run motion-compensated pipeline and compare to baseline')
    parser.add_argument('--search-radius',type=int,   default=8,
                        help='MC block search radius in pixels (default: 8)')
    parser.add_argument('--sad-threshold',type=int,   default=256,
                        help='SAD threshold for inter vs intra decision (default: 256)')
    parser.add_argument('--row',          type=int,   default=100)
    parser.add_argument('--col',          type=int,   default=160)
    parser.add_argument('--out',          default='wht_report.pdf')
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

        if args.mc:
            mc_cfg = {**cfg,
                      'search_radius': args.search_radius,
                      'sad_threshold': args.sad_threshold}
            print(f"\n--- Motion-Compensated  radius={args.search_radius}"
                  f"  SAD_thr={args.sad_threshold} ---\n")
            mc_results = analyze_frame_sequence_mc(frames, **mc_cfg)

            print(f"{'Pair':<35} {'Base B':>7} {'MC B':>7} {'Saving':>7} "
                  f"{'MC Ratio':>9} {'MC FPS':>7} {'MC PSNR':>8} "
                  f"{'MVs':>5} {'Intra':>6}")
            print("-" * 100)
            for base, mc in zip(seq_results, mc_results):
                saving   = base['compressed_bytes'] - mc['compressed_bytes']
                psnr_s   = f"{mc['psnr']:.1f}dB" if mc['psnr'] != float('inf') else "   inf"
                print(f"{mc['pair']:<35} {base['compressed_bytes']:>7} "
                      f"{mc['compressed_bytes']:>7} {saving:>+7} "
                      f"{mc['ratio']:>9.1f}x {mc['sf7_fps']:>7.2f} "
                      f"{psnr_s:>8} {mc['mv_blocks']:>5} {mc['intra_blocks']:>6}")

            mc_b    = [r['compressed_bytes'] for r in mc_results]
            mean_mc = sum(mc_b) / len(mc_b)
            hits_mc = sum(1 for r in mc_results if r['ratio'] >= 220)
            base_b  = [r['compressed_bytes'] for r in seq_results]
            mean_base = sum(base_b) / len(base_b)
            print(f"\n  Baseline:  mean={mean_base:.0f}B  ratio={76800/mean_base:.1f}x  "
                  f">=220:1: {sum(1 for r in seq_results if r['ratio'] >= 220)}/{len(seq_results)}")
            print(f"  MC:        mean={mean_mc:.0f}B  ratio={76800/mean_mc:.1f}x  "
                  f">=220:1: {hits_mc}/{len(mc_results)}")
            print(f"  Reduction: {(1 - mean_mc/mean_base)*100:.1f}% fewer bytes on average")

        if args.report:
            print(f"\nGenerating PDF -> {args.out}")
            generate_report(val_results, seq_results, nf,
                            frame_dir or '.', cfg, args.out)

        if args.plot:
            generate_plots(frames, frame_dir, cfg)

        if args.sweep:
            from swp_wht import (swp_qstep, swp_topk, swp_threshold,
                                 swp_rd_curve, ensure_dir)
            dataset = os.path.basename(os.path.normpath(frame_dir)) or 'default'
            dest = os.path.join('plt', dataset, 'swp')
            ensure_dir(dest)
            print(f"\nRunning parameter sweeps -> {dest}/")
            swp_qstep(frames, dest, cfg)
            swp_topk(frames, dest, cfg)
            swp_threshold(frames, dest, cfg)
            swp_rd_curve(frames, dest, cfg)
    else:
        if args.report:
            print("Need >= 2 frames. Use --frames <dir>")

    if len(frames) >= 2 and not (args.report or args.plot or args.sweep):
        print("\nTip: --plot to generate plots  |  --sweep for parameter sweeps  |  --report for PDF")


if __name__ == "__main__":
    main()
