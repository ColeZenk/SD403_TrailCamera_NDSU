"""
PDF report generation for WHT compression test results.
"""

import os
from datetime import datetime


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
    ax.set_xticks(list(x)); ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)

    ax = axes[0,1]
    bar_c = ['#1a7a3a' if r >= 220 else '#cc6600' if r >= 50 else '#cc0000'
              for r in ratios]
    ax.bar(x, ratios, color=bar_c)
    ax.axhline(220, color='orange', linestyle='--', lw=1, label='220:1 target')
    ax.set_title('Compression Ratio', fontweight='bold')
    ax.set_ylabel('x')
    ax.set_xticks(list(x)); ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)

    ax = axes[1,0]
    fps_c = ['#1a7a3a' if f >= 1 else '#cc6600' for f in fps_v]
    ax.bar(x, fps_v, color=fps_c)
    ax.axhline(1, color='blue', linestyle='--', lw=1, label='1 FPS')
    ax.set_title('SF7 FPS', fontweight='bold')
    ax.set_ylabel('FPS')
    ax.set_xticks(list(x)); ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
    ax.legend(fontsize=7)

    ax = axes[1,1]
    ax.bar(x, psnr_v, color='#7a3a8a', alpha=0.85)
    ax.axhline(30, color='green', linestyle='--', lw=1, label='30dB perceptual threshold')
    ax.set_title('Reconstruction PSNR (vs ground truth frame B)', fontweight='bold')
    ax.set_ylabel('dB')
    ax.set_xticks(list(x)); ax.set_xticklabels(pairs, rotation=45, ha='right', fontsize=7)
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
