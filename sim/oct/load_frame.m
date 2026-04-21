function frame = load_frame(path, FRAME_W, FRAME_H)
% load_frame — read raw 320x240 grayscale from disk

  fid = fopen(path, 'rb');
  raw = fread(fid, [1, FRAME_W * FRAME_H], 'uint8');
  fclose(fid);
  assert(length(raw) == FRAME_W * FRAME_H, ...
         sprintf('Expected %d bytes, got %d', FRAME_W * FRAME_H, length(raw)));
  frame = reshape(raw, [FRAME_W, FRAME_H])';
end
