function P = iwht_2d(C)
% iwht_2d — Inverse 2D WHT: same butterfly, divide by 64
% Runs on ESP32-S3 (float OK)

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
