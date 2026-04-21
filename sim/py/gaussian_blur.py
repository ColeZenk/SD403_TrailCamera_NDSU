import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter  # for convolution blur

# -----------------------------
# Configuration
# -----------------------------
image_folder = "../ESP_Ecosystem/esp32cam/docs/testing/artifact/TP201/"
WIDTH = 320
HEIGHT = 240
discard = 5          # discard first few unstable frames
blur_sigma = 1.0     # Gaussian blur sigma (adjust as needed)

# -----------------------------
# Load frames
# -----------------------------
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

# -----------------------------
# Discard unstable startup frames
# -----------------------------
frames = frames[discard:]
print(f"After discarding first {discard} frames: {len(frames)} frames")

# -----------------------------
# Optional: inspect frame means
# -----------------------------
for i, frame in enumerate(frames):
    print(i, np.mean(frame))

# -----------------------------
# Remove global exposure shift
# -----------------------------
frame_means = np.mean(frames, axis=(1, 2), keepdims=True)
frames_zero_mean = frames - frame_means

# -----------------------------
# Apply Gaussian blur (convolution)
# -----------------------------
frames_blur = np.array([gaussian_filter(f, sigma=blur_sigma) for f in frames_zero_mean])

# -----------------------------
# Compute frame-to-frame differences
# -----------------------------
diffs = frames_blur[1:] - frames_blur[:-1]
all_diffs = diffs.flatten()

# -----------------------------
# Statistics
# -----------------------------
mean_diff = np.mean(all_diffs)
sigma = np.std(all_diffs)

print(f"Mean diff: {mean_diff:.6f}")
print(f"Temporal diff sigma: {sigma:.4f}")
print(f"Estimated per-frame sigma: {sigma / np.sqrt(2):.4f}")
print(f"Suggested threshold (3σ): {3*sigma:.4f}")

print(f"Min diff: {np.min(all_diffs)}")
print(f"Max diff: {np.max(all_diffs)}")

# -----------------------------
# Plot histogram
# -----------------------------
plt.hist(all_diffs, bins=100, density=True)
plt.title("Frame-to-Frame Diff Histogram (blurred & normalized)")
plt.xlabel("Pixel difference")
plt.ylabel("Density")
plt.show()
