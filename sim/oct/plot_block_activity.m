function plot_block_activity(fa, fb, SMAX, SMIN, SEQ_THRESH, THRESHOLD, ...
                            Q_SHIFT, TOP_K, pair_name, out_path)
% plot_block_activity -- heatmap of active blocks + overlay on frame

  D = diff_frames(fa, fb);
  [~, encoded] = encode_frame(D, SMAX, SMIN, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K);

  grid = zeros(30, 40);
  for i = 1:length(encoded)
    e  = encoded{i};
    br = e.by / 8 + 1;
    bc = e.bx / 8 + 1;
    grid(br, bc) = e.n;
  end

  figure('visible', 'off', 'position', [0 0 1400 500]);

  % heatmap
  subplot(1, 2, 1);
  imagesc(grid);
  colormap(hot(256));
  cb = colorbar();
  ylabel(cb, '# coefficients');
  axis image;
  xlabel('Block column (0-39)');
  ylabel('Block row (0-29)');
  title(sprintf('Block Activity -- %s\n(%d blocks transmitted)', ...
        pair_name, length(encoded)));

  % overlay on frame
  subplot(1, 2, 2);
  imagesc(double(fa));
  colormap(gray(256));
  caxis([0 255]);
  axis image off;
  hold on;
  for i = 1:length(encoded)
    e  = encoded{i};
    bx = e.bx;
    by = e.by;
    rectangle('Position', [bx+1, by+1, 8, 8], ...
              'EdgeColor', 'r', 'LineWidth', 0.5, ...
              'FaceColor', 'none');
  end
  hold off;
  title('Active blocks overlaid on Frame A');

  print(out_path, '-dpng', '-r150');
  close;
  printf('  Saved: %s\n', out_path);
end
