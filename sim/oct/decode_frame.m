function recon = decode_frame(frame_a, encoded, Q_SHIFT, GAUSS_SIGMA)
% decode_frame — dequantize, IWHT, Gaussian smooth, add to reference
% Runs on ESP32-S3 (float OK)

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
      C(r+1, c+1) = double(bitshift(int32(q), Q_SHIFT));
    end

    patch = iwht_2d(C);

    for r = 0:7
      for c = 0:7
        diff_recon(by + r + 1, bx + c + 1) = patch(r+1, c+1);
      end
    end
  end

  % Gaussian smooth (hand-rolled, no packages)
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
