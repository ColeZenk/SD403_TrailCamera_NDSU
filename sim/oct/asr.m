function y = asr(v, n)
% asr — arithmetic right shift
% Maps to Verilog >>> operator

  if v >= 0
    y = floor(v / (2^n));
  else
    y = -floor((-v) / (2^n));
  end
end
