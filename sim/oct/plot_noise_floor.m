function plot_noise_floor(all_maxac, threshold, out_path)
% plot_noise_floor -- histogram of precomputed max AC values per block

  sorted = sort(all_maxac);
  N   = length(sorted);
  p50 = sorted(round(N * 0.50));
  p95 = sorted(round(N * 0.95));

  figure('visible', 'off', 'position', [0 0 1000 500]);
  hist(all_maxac, 100);
  h = findobj(gca, 'Type', 'patch');
  set(h, 'FaceColor', [0.23 0.37 0.66], 'EdgeColor', 'none', 'FaceAlpha', 0.8);

  hold on;
  yl = ylim();
  plot([threshold threshold], yl, '--r', 'LineWidth', 2);
  plot([p95 p95], yl, '--', 'Color', [1.0 0.6 0.0], 'LineWidth', 1.5);
  plot([p50 p50], yl, '--g', 'LineWidth', 1.5);
  hold off;

  legend(sprintf('block_threshold = %d', threshold), ...
         sprintf('p95 = %d', p95), ...
         sprintf('p50 = %d', p50), ...
         'Location', 'northeast');
  xlabel('max |AC coefficient| per 8x8 block');
  ylabel('Count (blocks)');
  title('Noise Floor Distribution -- max AC per block across all pairs');
  grid on;

  print(out_path, '-dpng', '-r150');
  close;
  printf('  Saved: %s\n', out_path);
end
