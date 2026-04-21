function [nf, all_maxac] = noise_floor_stats(frames, SMAX, SMIN, SEQ_THRESH)
% noise_floor_stats — per-block p95 of max AC coeff across static frames

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
