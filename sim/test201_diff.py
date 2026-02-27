import os
import numpy as np
import matplotlib.pyplot as plt

image_folder = "../ESP_Ecosystem/esp32cam/docs/testing/artifact/TP201/"

WIDTH = 320
HEIGHT = 240
discard = 5

filenames = sorted(os.listdir(image_folder))
frames = []

for name in filenames:
    path = os.path.join(image_folder, name)

    with open(path, "rb") as f:
        data = np.fromfile(f, dtype=np.uint8)

    if data.size != WIDTH * HEIGHT:
        print(f"Skipping {name} (wrong size: {data.size})")
        continue

    frame = data.reshape((HEIGHT, WIDTH)).astype(np.float32)
    frames.append(frame)

frames = np.array(frames)
print(f"Loaded {len(frames)} frames")

frames = frames[discard:]
print(f"After discard: {len(frames)} frames")

for i, frame in enumerate(frames):
    print(i, np.mean(frame))
# Remove global brightness shift (exposure drift)
frame_means = np.mean(frames, axis=(1, 2), keepdims=True)
frames_zero_mean = frames - frame_means

diffs = frames_zero_mean[1:] - frames_zero_mean[:-1]
all_diffs = diffs.flatten()

mean_diff = np.mean(all_diffs)
sigma = np.std(all_diffs)

print(f"Mean diff: {mean_diff:.6f}")
print(f"Temporal diff sigma: {sigma:.4f}")
print(f"Estimated per-frame sigma: {sigma / np.sqrt(2):.4f}")
print(f"Suggested threshold (3σ): {3*sigma:.4f}")

plt.hist(all_diffs, bins=100, density=True)
plt.title("Frame-to-Frame Diff Histogram")
plt.show()
