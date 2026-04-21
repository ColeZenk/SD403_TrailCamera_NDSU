function y = wht8(x, SMAX, SMIN)
% wht8 — 8-point 1D WHT, combinational butterfly
% Maps directly to wht8.v: 3 stages, stride=1,2,4
% Pure add/sub, no multiply. Clamp to [SMIN, SMAX].

  % stage 1 — stride=1
  s1    = zeros(1, 8);
  s1(1) = x(1) + x(2);  s1(2) = x(1) - x(2);
  s1(3) = x(3) + x(4);  s1(4) = x(3) - x(4);
  s1(5) = x(5) + x(6);  s1(6) = x(5) - x(6);
  s1(7) = x(7) + x(8);  s1(8) = x(7) - x(8);
  s1 = max(min(s1, SMAX), SMIN);

  % stage 2 — stride=2
  s2    = zeros(1, 8);
  s2(1) = s1(1) + s1(3);  s2(3) = s1(1) - s1(3);
  s2(2) = s1(2) + s1(4);  s2(4) = s1(2) - s1(4);
  s2(5) = s1(5) + s1(7);  s2(7) = s1(5) - s1(7);
  s2(6) = s1(6) + s1(8);  s2(8) = s1(6) - s1(8);
  s2 = max(min(s2, SMAX), SMIN);

  % stage 3 — stride=4
  y    = zeros(1, 8);
  y(1) = s2(1) + s2(5);  y(5) = s2(1) - s2(5);
  y(2) = s2(2) + s2(6);  y(6) = s2(2) - s2(6);
  y(3) = s2(3) + s2(7);  y(7) = s2(3) - s2(7);
  y(4) = s2(4) + s2(8);  y(8) = s2(4) - s2(8);
  y = max(min(y, SMAX), SMIN);
end
