function [names, frames] = load_frame_dir(dir_path, FRAME_W, FRAME_H)
% load_frame_dir — load all img_NNNN.raw files from directory, sorted

  files = dir(fullfile(dir_path, 'img_*.raw'));
  [~, idx] = sort({files.name});
  files = files(idx);
  names  = {};
  frames = {};
  for i = 1:length(files)
    path = fullfile(dir_path, files(i).name);
    try
      f = load_frame(path, FRAME_W, FRAME_H);
      names{end+1}  = files(i).name;
      frames{end+1} = f;
    catch e
      printf('  Skipping %s: %s\n', files(i).name, e.message);
    end
  end
end
