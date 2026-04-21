function C = wht_2d(block, SMAX, SMIN)
% wht_2d — 2D WHT: row pass then col pass
% Maps to wht_2d.v FSM

  C = zeros(8, 8);
  for r = 1:8
    C(r, :) = wht8(block(r, :), SMAX, SMIN);
  end
  for c = 1:8
    C(:, c) = wht8(C(:, c)', SMAX, SMIN)';
  end
end
