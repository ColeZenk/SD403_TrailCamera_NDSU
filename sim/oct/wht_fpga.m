% wht_fpga.m — Full WHT compression/reconstruction pipeline
%
% Usage:
%   octave --no-gui wht_fpga.m                          % synthetic test only
%   octave --no-gui wht_fpga.m path/to/frames/          % real frames
%   octave --no-gui wht_fpga.m path/to/frames/ --plot   % + reconstruction PNGs

% =====================================================================
% Constants — match FPGA config registers
% =====================================================================
W = 18;
SMAX =  (2^(W-1)) - 1;
SMIN = -(2^(W-1));

FRAME_W = 320;
FRAME_H = 240;

SEQ_THRESH  = 2;
THRESHOLD   = 400;
Q_SHIFT     = 5;        % q_step = 2^5 = 32
TOP_K       = 5;
GAUSS_SIGMA = 4.0;

% =====================================================================
% WHT validation
% =====================================================================
printf('Walsh-Hadamard Transform Validation\n');
printf('============================================================\n');

for i = 1:20
  x = randi([-128, 127], 1, 8);
  C = wht8(x, SMAX, SMIN);
  R = wht8(C, SMAX, SMIN);
  err_1d = max(abs(double(x) - double(R) / 8));
  assert(err_1d < 0.01, sprintf('1D round-trip failed, err=%.4f', err_1d));
end
printf('1D round-trip: 20 random vectors PASS\n');

for i = 1:10
  test_block = randi([-50, 50], 8, 8);
  C2 = wht_2d(test_block, SMAX, SMIN);
  P2 = iwht_2d(C2);
  err_2d = max(max(abs(double(test_block) - P2)));
  assert(err_2d < 0.02, sprintf('2D round-trip failed, err=%.6f', err_2d));
end
printf('2D round-trip: 10 random blocks PASS\n');
printf('============================================================\n');

% =====================================================================
% Load frames
% =====================================================================
args = argv();
do_plot = any(strcmp(args, '--plot'));
args = args(~strcmp(args, '--plot'));   % strip flag

if length(args) >= 1
  frame_dir = args{1};
else
  candidates = {'../esp/esp32cam/docs/testing/artifact/test3', ...
                '../esp/esp32cam/docs/testing/artifact/test4', ...
                './frames'};
  frame_dir = '';
  for i = 1:length(candidates)
    if ~isempty(dir(fullfile(candidates{i}, 'img_*.raw')))
      frame_dir = candidates{i};
      break;
    end
  end
end

if isempty(frame_dir) || ~isfolder(frame_dir)
  printf('\nNo frame directory found.\n');
  printf('Usage: octave --no-gui wht_fpga.m path/to/frames/\n');
  return;
end

[names, frames] = load_frame_dir(frame_dir, FRAME_W, FRAME_H);
n_frames = length(frames);
printf('\nLoaded %d frames from %s\n', n_frames, frame_dir);

if n_frames < 2
  printf('Need >= 2 frames.\n');
  return;
end

% =====================================================================
% Demo block
% =====================================================================
D0 = diff_frames(frames{1}, frames{2});
demo_r = 100; demo_c = 160;
block = double(D0(demo_r+1:demo_r+8, demo_c+1:demo_c+8));
C = wht_2d(block, SMAX, SMIN);
M = sequency_mask(C, SEQ_THRESH);

printf('\nDemo 8x8 block  row=%d col=%d\n', demo_r, demo_c);
printf('%-4s %-50s %-50s\n', 'Row', '--- diff ---', '--- WHT coefficients ---');
printf('%s\n', repmat('-', 1, 100));
for r = 1:8
  px_str = '';
  co_str = '';
  for c = 1:8
    px_str = [px_str, sprintf('%5d', block(r, c))];
    co_str = [co_str, sprintf('%7d', C(r, c))];
  end
  printf('%-4d%s   %s\n', r-1, px_str, co_str);
end

n_kept  = 0;
total_e = 0;
kept_e  = 0;
for r = 0:7
  for c = 0:7
    total_e = total_e + C(r+1, c+1)^2;
    if (r + c) <= SEQ_THRESH
      n_kept = n_kept + 1;
      kept_e = kept_e + M(r+1, c+1)^2;
    end
  end
end
printf('\n  seq_thresh=%d: keeping %d/64 coefficients\n', SEQ_THRESH, n_kept);
if total_e > 0
  printf('  Energy retained: %.1f%%\n', 100 * kept_e / total_e);
end

% =====================================================================
% Single pair round-trip
% =====================================================================
printf('\n--- Round-trip  Q=%d  K=%d  seq=%d  sigma=%.1f ---\n', ...
       2^Q_SHIFT, TOP_K, SEQ_THRESH, GAUSS_SIGMA);
[b, ratio, psnr, n_changed, ~] = encode_decode( ...
    frames{1}, frames{2}, SMAX, SMIN, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K, GAUSS_SIGMA);
printf('  Changed blocks:  %d (%.1f%%)\n', n_changed, 100*n_changed/1200);
printf('  Compressed:      %d bytes\n', b);
printf('  Ratio:           %.1fx\n', ratio);
printf('  SF7 FPS:         %.2f\n', 683 / max(b, 1));
printf('  PSNR:            %.1f dB\n', psnr);

% =====================================================================
% Full sequence analysis
% =====================================================================
printf('\n--- Sequence Analysis  %d frames / %d pairs ---\n', n_frames, n_frames-1);
printf('    Q=%d  K=%d  seq=%d  thr=%d  sigma=%.1f\n\n', ...
       2^Q_SHIFT, TOP_K, SEQ_THRESH, THRESHOLD, GAUSS_SIGMA);

printf('%-35s %7s %8s %6s %8s %8s\n', 'Pair', 'Bytes', 'Ratio', 'FPS', 'PSNR', 'Changed');
printf('%s\n', repmat('-', 1, 78));

all_bytes = zeros(1, n_frames - 1);
all_psnr  = zeros(1, n_frames - 1);
all_recon = {};
for fi = 1 : n_frames - 1
  [b, ratio, psnr, n_ch, recon] = encode_decode( ...
      frames{fi}, frames{fi+1}, SMAX, SMIN, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K, GAUSS_SIGMA);
  all_bytes(fi) = b;
  all_psnr(fi)  = psnr;
  if do_plot
    all_recon{fi} = recon;
  end
  fps = 683 / max(b, 1);

  pair_name = sprintf('%s->>%s', names{fi}, names{fi+1});
  if isinf(psnr)
    psnr_s = '   inf';
  else
    psnr_s = sprintf('%.1fdB', psnr);
  end
  printf('%-35s %7d %7.1fx %6.2f %8s %8d\n', pair_name, b, ratio, fps, psnr_s, n_ch);
end

mean_b = mean(all_bytes);
hits   = sum(all_bytes <= 76800 ./ 220);
printf('\n  mean=%.0fB  mean_ratio=%.1fx  >=220:1: %d/%d  min=%dB  max=%dB\n', ...
       mean_b, 76800/mean_b, hits, n_frames-1, min(all_bytes), max(all_bytes));

% =====================================================================
% Noise floor
% =====================================================================
[nf, nf_raw] = noise_floor_stats(frames, SMAX, SMIN, SEQ_THRESH);
printf('\n  Noise floor  p25=%d  p50=%d  p75=%d  p95=%d\n', ...
       nf.p25, nf.p50, nf.p75, nf.p95);
if THRESHOLD > nf.p95
  printf('  Threshold %d is above p95\n', THRESHOLD);
else
  printf('  Threshold %d is BELOW p95\n', THRESHOLD);
end

% =====================================================================
% Plots (--plot flag)
% =====================================================================
if do_plot
  [~, dataset] = fileparts(frame_dir);
  if isempty(dataset); dataset = 'default'; end
  plt_dir = fullfile('plt', dataset);
  mkdir(plt_dir);

  cfg.q_shift    = Q_SHIFT;
  cfg.top_k      = TOP_K;
  cfg.seq_thresh = SEQ_THRESH;
  cfg.threshold  = THRESHOLD;
  cfg.gauss_sigma = GAUSS_SIGMA;

  printf('\n--- Generating plots -> %s/ ---\n', plt_dir);

  % [1/4] noise floor histogram
  printf('  [1/4] Noise floor histogram...\n');
  plot_noise_floor(nf_raw, THRESHOLD, fullfile(plt_dir, 'noise_floor.png'));

  % [2/4] sequence overview
  printf('  [2/4] Sequence overview...\n');
  plot_sequence(all_bytes, all_psnr, names, cfg, fullfile(plt_dir, 'sequence.png'));

  % [3/4] per-pair reconstruction + error + block activity
  [~, idx_worst] = max(all_bytes);
  [~, idx_best]  = min(all_bytes);
  plot_pairs = unique([1, idx_worst, idx_best, n_frames-1]);

  for fi = plot_pairs
    pair_name = sprintf('%s->>%s', names{fi}, names{fi+1});
    printf('  [3/4] Pair %d: %s\n', fi, pair_name);

    fname = sprintf('recon_%03d.png', fi);
    plot_recon(frames{fi}, frames{fi+1}, all_recon{fi}, pair_name, cfg, ...
              fullfile(plt_dir, fname));

    fname = sprintf('error_%03d.png', fi);
    plot_error_map(frames{fi}, frames{fi+1}, all_recon{fi}, pair_name, cfg, ...
                   fullfile(plt_dir, fname));

    fname = sprintf('blocks_%03d.png', fi);
    plot_block_activity(frames{fi}, frames{fi+1}, SMAX, SMIN, ...
                        SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K, ...
                        pair_name, fullfile(plt_dir, fname));
  end

  printf('  [4/4] Done.\n');
end
