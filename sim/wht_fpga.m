% wht_fpga.m — Full WHT compression/reconstruction pipeline in Octave
%
% Equivalent to the Python sim (codec.py + wht_core.py + frames.py).
% All encode-side arithmetic is fixed-point signed integer, W=18 bits.
% Decode side uses float (runs on ESP32-S3, not FPGA).
%
% Usage:
%   octave --no-gui wht_fpga.m                          % synthetic test
%   octave --no-gui wht_fpga.m path/to/frames/          % real frames

% =====================================================================
% Constants
% =====================================================================
W = 18;
SMAX =  (2^(W-1)) - 1;
SMIN = -(2^(W-1));

FRAME_W = 320;
FRAME_H = 240;
BLOCK   = 8;
BLOCKS_X = FRAME_W / BLOCK;   % 40
BLOCKS_Y = FRAME_H / BLOCK;   % 30

SEQ_THRESH = 2;
THRESHOLD  = 400;
Q_SHIFT    = 5;
TOP_K      = 5;
GAUSS_SIGMA = 4.0;


% =====================================================================
%  wht8 — 8-point 1D WHT, combinational butterfly (-> wht8.v)
% =====================================================================
function y = wht8(x, SMAX, SMIN)
  s1    = zeros(1, 8);
  s1(1) = x(1) + x(2);  s1(2) = x(1) - x(2);
  s1(3) = x(3) + x(4);  s1(4) = x(3) - x(4);
  s1(5) = x(5) + x(6);  s1(6) = x(5) - x(6);
  s1(7) = x(7) + x(8);  s1(8) = x(7) - x(8);
  s1 = max(min(s1, SMAX), SMIN);

  s2    = zeros(1, 8);
  s2(1) = s1(1) + s1(3);  s2(3) = s1(1) - s1(3);
  s2(2) = s1(2) + s1(4);  s2(4) = s1(2) - s1(4);
  s2(5) = s1(5) + s1(7);  s2(7) = s1(5) - s1(7);
  s2(6) = s1(6) + s1(8);  s2(8) = s1(6) - s1(8);
  s2 = max(min(s2, SMAX), SMIN);

  y    = zeros(1, 8);
  y(1) = s2(1) + s2(5);  y(5) = s2(1) - s2(5);
  y(2) = s2(2) + s2(6);  y(6) = s2(2) - s2(6);
  y(3) = s2(3) + s2(7);  y(7) = s2(3) - s2(7);
  y(4) = s2(4) + s2(8);  y(8) = s2(4) - s2(8);
  y = max(min(y, SMAX), SMIN);
end


% =====================================================================
%  wht_2d — 2D WHT: row pass then col pass (-> wht_2d.v)
% =====================================================================
function C = wht_2d(block, SMAX, SMIN)
  C = zeros(8, 8);
  for r = 1:8
    C(r, :) = wht8(block(r, :), SMAX, SMIN);
  end
  for c = 1:8
    C(:, c) = wht8(C(:, c)', SMAX, SMIN)';
  end
end


% =====================================================================
%  iwht_2d — Inverse 2D WHT: same butterfly, divide by 64 (ESP32-S3)
% =====================================================================
function P = iwht_2d(C)
  INF_MAX = realmax;
  INF_MIN = -realmax;
  T = zeros(8, 8);
  for r = 1:8
    T(r, :) = wht8(C(r, :), INF_MAX, INF_MIN);
  end
  for c = 1:8
    T(:, c) = wht8(T(:, c)', INF_MAX, INF_MIN)';
  end
  P = T / 64.0;
end


% =====================================================================
%  asr — arithmetic right shift (-> Verilog >>>)
% =====================================================================
function y = asr(v, n)
  if v >= 0
    y = floor(v / (2^n));
  else
    y = -floor((-v) / (2^n));
  end
end


% =====================================================================
%  Frame I/O — load raw 320x240 grayscale (-> frames.py)
% =====================================================================
function frame = load_frame(path, FRAME_W, FRAME_H)
  fid = fopen(path, 'rb');
  raw = fread(fid, [1, FRAME_W * FRAME_H], 'uint8');
  fclose(fid);
  assert(length(raw) == FRAME_W * FRAME_H, ...
         sprintf('Expected %d bytes, got %d', FRAME_W * FRAME_H, length(raw)));
  frame = reshape(raw, [FRAME_W, FRAME_H])';  % row-major: (H, W)
end


function [names, frames] = load_frame_dir(dir_path, FRAME_W, FRAME_H)
  files = dir(fullfile(dir_path, 'img_*.raw'));
  [~, idx] = sort({files.name});
  files = files(idx);
  names  = {};
  frames = {};
  for i = 1:length(files)
    path = fullfile(dir_path, files(i).name);
    try
      f = load_frame(path, FRAME_W, FRAME_H);
      names{end+1}  = files(i).name;
      frames{end+1} = f;
    catch e
      printf('  Skipping %s: %s\n', files(i).name, e.message);
    end
  end
end


% =====================================================================
%  diff_frames — inter-frame difference (integer)
% =====================================================================
function D = diff_frames(fa, fb)
  D = int32(fb) - int32(fa);
end


% =====================================================================
%  sequency_mask — zero coefficients where r+c > thresh
% =====================================================================
function M = sequency_mask(C, seq_thresh)
  M = C;
  for r = 0:7
    for c = 0:7
      if (r + c) > seq_thresh
        M(r+1, c+1) = 0;
      end
    end
  end
end


% =====================================================================
%  coeff_out — threshold, quantize, top-K select (-> coeff_out.v)
% =====================================================================
function [n_out, out] = coeff_out(C, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K)
  buf = zeros(64, 3);
  n   = 0;

  for r = 0:7
    for c = 0:7
      if (r + c) > SEQ_THRESH;  continue;  end
      if r == 0 && c == 0;     continue;  end

      v   = C(r+1, c+1);
      mag = abs(v);
      if mag <= THRESHOLD;  continue;  end

      if v > 0
        att = mag - THRESHOLD;
      else
        att = -(mag - THRESHOLD);
      end

      q = asr(att, Q_SHIFT);
      if q == 0;  continue;  end

      n = n + 1;
      buf(n, :) = [r, c, q];
    end
  end

  if n > 0
    [~, idx] = sort(abs(buf(1:n, 3)), 'descend');
    k   = min(n, TOP_K);
    out = buf(idx(1:k), :);
    n_out = k;
  else
    out   = [];
    n_out = 0;
  end
end


% =====================================================================
%  encode_frame — full frame encode (-> FPGA top-level pipeline)
% =====================================================================
function [total_bytes, encoded] = encode_frame(diff, SMAX, SMIN, ...
                                    SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K)
  BLOCKS_Y = rows(diff)    / 8;
  BLOCKS_X = columns(diff) / 8;

  total_bytes = 2;    % frame header
  encoded = {};       % cell array of {by, bx, coeffs_matrix}

  for by = 0 : BLOCKS_Y - 1
    for bx = 0 : BLOCKS_X - 1
      r0 = by * 8 + 1;
      c0 = bx * 8 + 1;
      block = double(diff(r0:r0+7, c0:c0+7));

      C = wht_2d(block, SMAX, SMIN);
      M = sequency_mask(C, SEQ_THRESH);

      % ac_max — skip DC
      ac_max = 0;
      for r = 0:7
        for c = 0:7
          if r == 0 && c == 0;  continue;  end
          if (r + c) > SEQ_THRESH;  continue;  end
          if abs(M(r+1, c+1)) > ac_max
            ac_max = abs(M(r+1, c+1));
          end
        end
      end
      if ac_max < THRESHOLD;  continue;  end

      [n_coeffs, coeffs] = coeff_out(C, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K);
      if n_coeffs == 0;  continue;  end

      total_bytes = total_bytes + 2 + n_coeffs + 1;  % coords + coeffs + EOB
      encoded{end+1} = struct('by', by*8, 'bx', bx*8, ...
                              'coeffs', coeffs, 'n', n_coeffs);
    end
  end

  total_bytes = min(total_bytes, 76800);
end


% =====================================================================
%  decode_frame — dequantize, IWHT, Gaussian smooth, add ref (ESP32-S3)
% =====================================================================
function recon = decode_frame(frame_a, encoded, Q_SHIFT, GAUSS_SIGMA)
  [H, W_] = size(frame_a);
  diff_recon = zeros(H, W_);

  for i = 1:length(encoded)
    e  = encoded{i};
    by = e.by;
    bx = e.bx;

    C = zeros(8, 8);
    for j = 1:e.n
      r = e.coeffs(j, 1);
      c = e.coeffs(j, 2);
      q = e.coeffs(j, 3);
      C(r+1, c+1) = double(bitshift(int32(q), Q_SHIFT));  % q << Q_SHIFT
    end

    patch = iwht_2d(C);

    for r = 0:7
      for c = 0:7
        diff_recon(by + r + 1, bx + c + 1) = patch(r+1, c+1);
      end
    end
  end

  % Gaussian smooth (hand-rolled, no image pkg dependency)
  if GAUSS_SIGMA > 0
    k_half = ceil(GAUSS_SIGMA * 3);
    k_size = 2 * k_half + 1;
    [gx, gy] = meshgrid(-k_half:k_half, -k_half:k_half);
    kernel = exp(-(gx.^2 + gy.^2) / (2 * GAUSS_SIGMA^2));
    kernel = kernel / sum(kernel(:));
    diff_recon = conv2(diff_recon, kernel, 'same');
  end

  recon = double(frame_a) + diff_recon;
  recon = max(min(recon, 255), 0);
end


% =====================================================================
%  encode_decode — full round-trip with PSNR (-> codec.py:encode_decode)
% =====================================================================
function [bytes_out, ratio, psnr, n_changed, recon] = ...
    encode_decode(fa, fb, SMAX, SMIN, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K, GAUSS_SIGMA)

  D = diff_frames(fa, fb);
  [bytes_out, encoded] = encode_frame(D, SMAX, SMIN, ...
                              SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K);
  recon = decode_frame(fa, encoded, Q_SHIFT, GAUSS_SIGMA);

  n_changed = length(encoded);
  ratio     = 76800 / max(bytes_out, 1);

  fb_d = double(fb);
  mse  = mean((fb_d(:) - recon(:)).^2);
  if mse > 0
    psnr = 10 * log10(255^2 / mse);
  else
    psnr = Inf;
  end
end


% =====================================================================
%  noise_floor_stats — per-block p95 of max AC coeff (static frames)
% =====================================================================
function nf = noise_floor_stats(frames, SMAX, SMIN, SEQ_THRESH)
  n_frames = length(frames);
  all_maxac = [];

  for fi = 1 : n_frames - 1
    D = diff_frames(frames{fi}, frames{fi+1});
    for by = 0:29
      for bx = 0:39
        r0 = by * 8 + 1;
        c0 = bx * 8 + 1;
        block = double(D(r0:r0+7, c0:c0+7));
        C = wht_2d(block, SMAX, SMIN);
        ac_max = 0;
        for r = 0:7
          for c = 0:7
            if r == 0 && c == 0;  continue;  end
            if abs(C(r+1, c+1)) > ac_max
              ac_max = abs(C(r+1, c+1));
            end
          end
        end
        all_maxac(end+1) = ac_max;
      end
    end
  end

  all_maxac = sort(all_maxac);
  N = length(all_maxac);
  nf.p25 = all_maxac(round(N * 0.25));
  nf.p50 = all_maxac(round(N * 0.50));
  nf.p75 = all_maxac(round(N * 0.75));
  nf.p95 = all_maxac(round(N * 0.95));
  nf.max = all_maxac(N);
end


% #####################################################################
%  MAIN
% #####################################################################

% No external packages required — all math is hand-rolled

% --- WHT validation ---
printf('Walsh-Hadamard Transform Validation\n');
printf('============================================================\n');

% 1D round-trip: WHT then IWHT = x * N, so x = IWHT(WHT(x)) / N
for i = 1:20
  x = randi([-128, 127], 1, 8);
  C = wht8(x, SMAX, SMIN);
  R = wht8(C, SMAX, SMIN);         % same transform for inverse
  err_1d = max(abs(double(x) - double(R) / 8));
  assert(err_1d < 0.01, sprintf('1D round-trip failed, err=%.4f', err_1d));
end
printf('1D round-trip: 20 random vectors PASS\n');

% 2D round-trip
for i = 1:10
  test_block = randi([-50, 50], 8, 8);
  C2 = wht_2d(test_block, SMAX, SMIN);
  P2 = iwht_2d(C2);
  err_2d = max(max(abs(double(test_block) - P2)));
  assert(err_2d < 0.02, sprintf('2D round-trip failed, err=%.6f', err_2d));
end
printf('2D round-trip: 10 random blocks PASS (max err < 0.02)\n');
printf('============================================================\n');

% --- Load frames ---
args = argv();
if length(args) >= 1
  frame_dir = args{1};
else
  % auto-detect
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
  printf('\nNo frame directory found. Exiting.\n');
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

% --- Demo block ---
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

% --- Single pair round-trip ---
printf('\n--- Round-trip  Q=%d  K=%d  seq=%d  sigma=%.1f ---\n', ...
       2^Q_SHIFT, TOP_K, SEQ_THRESH, GAUSS_SIGMA);
[b, ratio, psnr, n_changed, ~] = encode_decode( ...
    frames{1}, frames{2}, SMAX, SMIN, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K, GAUSS_SIGMA);
printf('  Changed blocks:  %d (%.1f%%)\n', n_changed, 100*n_changed/1200);
printf('  Compressed:      %d bytes\n', b);
printf('  Ratio:           %.1fx\n', ratio);
printf('  SF7 FPS:         %.2f\n', 683 / max(b, 1));
printf('  PSNR:            %.1f dB\n', psnr);

% --- Full sequence analysis ---
printf('\n--- Sequence Analysis  %d frames / %d pairs ---\n', n_frames, n_frames-1);
printf('    Q=%d  K=%d  seq=%d  thr=%d  sigma=%.1f\n\n', ...
       2^Q_SHIFT, TOP_K, SEQ_THRESH, THRESHOLD, GAUSS_SIGMA);

printf('%-35s %7s %8s %6s %8s %8s\n', 'Pair', 'Bytes', 'Ratio', 'FPS', 'PSNR', 'Changed');
printf('%s\n', repmat('-', 1, 78));

all_bytes = zeros(1, n_frames - 1);
for fi = 1 : n_frames - 1
  [b, ratio, psnr, n_ch, ~] = encode_decode( ...
      frames{fi}, frames{fi+1}, SMAX, SMIN, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K, GAUSS_SIGMA);
  all_bytes(fi) = b;
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

% --- Noise floor ---
nf = noise_floor_stats(frames, SMAX, SMIN, SEQ_THRESH);
printf('\n  Noise floor  p25=%d  p50=%d  p75=%d  p95=%d\n', ...
       nf.p25, nf.p50, nf.p75, nf.p95);
if THRESHOLD > nf.p95
  printf('  Threshold %d is above p95\n', THRESHOLD);
else
  printf('  Threshold %d is BELOW p95\n', THRESHOLD);
end
