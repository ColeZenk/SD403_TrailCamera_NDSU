function M = sequency_mask(C, seq_thresh)
% sequency_mask — zero coefficients where r+c > thresh
% Single comparator in hardware

  M = C;
  for r = 0:7
    for c = 0:7
      if (r + c) > seq_thresh
        M(r+1, c+1) = 0;
      end
    end
  end
end
