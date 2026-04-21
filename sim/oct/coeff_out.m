function [n_out, out] = coeff_out(C, SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K)
% coeff_out — threshold, quantize, top-K select
% Maps to coeff_out.v: comparators + shift + serialize

  buf = zeros(64, 3);
  n   = 0;

  for r = 0:7
    for c = 0:7
      if (r + c) > SEQ_THRESH;  continue;  end
      if r == 0 && c == 0;     continue;  end

      v   = C(r+1, c+1);
      mag = abs(v);
      if mag <= THRESHOLD;  continue;  end

      % soft threshold: subtract toward zero
      if v > 0
        att = mag - THRESHOLD;
      else
        att = -(mag - THRESHOLD);
      end

      % quantize: arithmetic right shift
      q = asr(att, Q_SHIFT);
      if q == 0;  continue;  end

      n = n + 1;
      buf(n, :) = [r, c, q];
    end
  end

  % sort by |q| descending, take top_k
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
