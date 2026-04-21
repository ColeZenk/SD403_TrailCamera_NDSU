"""
Frame I/O — load raw grayscale frames from disk.
"""

import os
import glob


PIXELS_X = 320
PIXELS_Y = 240


def load_frame(data_path):
    with open(data_path, 'rb') as f:
        data = f.read()
        assert len(data) == PIXELS_X * PIXELS_Y, \
            f"Expected {PIXELS_X*PIXELS_Y}, got {len(data)} in {data_path}"
        sample_frame = []
        for y in range(PIXELS_Y):
            row = data[y*PIXELS_X : y*PIXELS_X + PIXELS_X]
            sample_frame.append(row)
    return sample_frame


def load_frame_dir(directory):
    """Load all img_NNNN.raw files from a directory, sorted."""
    paths = sorted(glob.glob(os.path.join(directory, 'img_*.raw')))
    frames = []
    for p in paths:
        try:
            frames.append((os.path.basename(p), load_frame(p)))
        except AssertionError as e:
            print(f"  Skipping {p}: {e}")
    return frames
