function [total_bytes, encoded] = encode_frame(diff, SMAX, SMIN, ...
                                    SEQ_THRESH, THRESHOLD, Q_SHIFT, TOP_K)
% encode_frame — full frame encode, FPGA top-level pipeline
% Fixed-point integer arithmetic throughout.

  BLOCKS_Y = rows(diff)    / 8;
  BLOCKS_X = columns(diff) / 8;

  total_bytes = 2;
  encoded = {};

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

      total_bytes = total_bytes + 2 + n_coeffs + 1;
      encoded{end+1} = struct('by', by*8, 'bx', bx*8, ...
                              'coeffs', coeffs, 'n', n_coeffs);
    end
  end

  total_bytes = min(total_bytes, 76800);
end
