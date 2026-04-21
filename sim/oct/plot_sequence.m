function plot_sequence(all_bytes, all_psnr, names, cfg, out_path)
% plot_sequence — dual y-axis: bytes and PSNR across frame pairs

  n = length(all_bytes);
  x = 1:n;

  figure('visible', 'off', 'position', [0 0 1200 500]);

  % left axis: bytes (bar)
  ax1 = axes();
  bar(ax1, x, all_bytes, 0.6, 'FaceColor', [0.3 0.5 0.8]);
  ylabel(ax1, 'Compressed bytes');
  ylim(ax1, [0, max(all_bytes) * 1.2]);
  set(ax1, 'XTick', x(1:2:end));
  xlabel(ax1, 'Frame pair');

  % LoRa budget line
  hold(ax1, 'on');
  plot(ax1, [0 n+1], [683 683], '--', 'Color', [0.2 0.7 0.2], 'LineWidth', 1.5);
  hold(ax1, 'off');

  % right axis: PSNR (line) — overlay with transparent background
  ax2 = axes('Position', get(ax1, 'Position'), ...
             'YAxisLocation', 'right', ...
             'Color', 'none', ...
             'XTick', [], ...
             'XLim', get(ax1, 'XLim'));
  hold(ax2, 'on');
  plot(ax2, x, all_psnr, '-o', 'Color', [0.8 0.2 0.2], 'LineWidth', 1.5, 'MarkerSize', 4);
  ylabel(ax2, 'PSNR (dB)');
  ylim(ax2, [min(all_psnr) * 0.9, max(all_psnr) * 1.1]);
  hold(ax2, 'off');

  title(ax1, sprintf('Sequence Analysis    Q=%d  K=%d  seq=%d  thr=%d', ...
        2^cfg.q_shift, cfg.top_k, cfg.seq_thresh, cfg.threshold));

  legend(ax1, 'Bytes', 'LoRa budget (683B)', 'Location', 'northwest');

  print(out_path, '-dpng', '-r150');
  close;
  printf('  Saved: %s\n', out_path);
end
