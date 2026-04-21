function [bytes_out, ratio, psnr, n_changed, recon] = ...
    encode_decode(fa, fb, SMAX, SMIN, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K, GAUSS_SIGMA)
% encode_decode — full round-trip with PSNR

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
