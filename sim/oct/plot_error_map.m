function plot_error_map(fa, fb, recon, pair_name, cfg, out_path)
% plot_error_map -- |ground_truth - reconstructed| heatmap

  fb_d  = double(fb);
  error = abs(fb_d - recon);

  psnr_val = 10 * log10(255^2 / mean((fb_d(:) - recon(:)).^2));

  figure('visible', 'off', 'position', [0 0 1500 450]);

  subplot(1, 3, 1);
  imagesc(fb_d);
  colormap(gray(256));
  caxis([0 255]);
  axis image off;
  title('Ground Truth B');

  subplot(1, 3, 2);
  imagesc(recon);
  colormap(gray(256));
  caxis([0 255]);
  axis image off;
  title(sprintf('Reconstructed B  (PSNR=%.1fdB)', psnr_val));

  subplot(1, 3, 3);
  imagesc(error);
  colormap(hot(256));
  cb = colorbar();
  ylabel(cb, 'Absolute error (gray levels)');
  axis image off;
  title(sprintf('|Error|  (max=%.1f, mean=%.1f)', max(error(:)), mean(error(:))));

  axes('Position', [0 0 1 1], 'Visible', 'off');
  text(0.5, 0.98, sprintf('%s    Q=%d  K=%d', ...
       pair_name, 2^cfg.q_shift, cfg.top_k), ...
       'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');

  print(out_path, '-dpng', '-r150');
  close;
  printf('  Saved: %s\n', out_path);
end
