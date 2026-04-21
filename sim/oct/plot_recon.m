function plot_recon(fa, fb, recon, pair_name, cfg, out_path)
% plot_recon — side-by-side: reference, ground truth, reconstructed
%
% fa, fb: raw frames (H x W)
% recon:  reconstructed frame (H x W)
% cfg:    struct with q_shift, top_k, seq_thresh, threshold, gauss_sigma
% out_path: save path (PNG)

  psnr_val = 10 * log10(255^2 / mean((double(fb(:)) - recon(:)).^2));
  bytes    = 0;  % caller can overlay if needed

  figure('visible', 'off', 'position', [0 0 1500 500]);

  subplot(1, 3, 1);
  imagesc(double(fa));
  colormap(gray(256));
  caxis([0 255]);
  axis image off;
  title('Frame A (reference)');

  subplot(1, 3, 2);
  imagesc(double(fb));
  colormap(gray(256));
  caxis([0 255]);
  axis image off;
  title('Frame B (ground truth)');

  subplot(1, 3, 3);
  imagesc(recon);
  colormap(gray(256));
  caxis([0 255]);
  axis image off;
  title(sprintf('Reconstructed  %.1f dB', psnr_val));

  axes('Position', [0 0 1 1], 'Visible', 'off');
  text(0.5, 0.98, sprintf('%s    Q=%d  K=%d  seq=%d  sigma=%.1f', ...
       pair_name, 2^cfg.q_shift, cfg.top_k, cfg.seq_thresh, cfg.gauss_sigma), ...
       'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');

  print(out_path, '-dpng', '-r150');
  close;
  printf('  Saved: %s\n', out_path);
end
