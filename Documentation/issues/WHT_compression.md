# WHT-Based Image Compression Pipeline

## Summary
Implement a Walsh-Hadamard Transform based inter-frame compression pipeline for transmitting wildlife motion events over LoRa at usable framerates. The algorithm exploits sparsity in frame differences to achieve high compression ratios with minimal compute and no multipliers.

## Background
Standard image codecs (JPEG, H.264) are too compute-heavy for our hardware budget. The WHT is a natural fit because:
- **Multiplier-free** — only adds and subtracts, maps directly to FPGA butterfly hardware
- **Inter-frame differencing** eliminates static background entirely
- **Walsh domain sparsity** — motion events concentrate energy in a few coefficients
- **DC term isolation** — global brightness shifts (exposure drift) appear only in DC and can be ignored

Validated on real ESP32-CAM frames. Static background blocks show >99% energy in DC. Motion diff blocks show <1% energy in DC with significant AC coefficients representing real motion.

## Algorithm Overview

```
Frame N+1 arrives
    → Diff against reference frame N
    → Tile diff into 8×8 blocks
    → WHT each changed block (branchless butterfly, no multipliers)
    → Threshold AC coefficients against noise floor
    → Quantize and pack top-N coefficients
    → Transmit over LoRa
Phone side:
    → Unpack coefficients
    → Inverse WHT per block
    → Apply diff to reference frame
    → Display reconstructed frame
```

## Implementation Phases

### Phase 1 — Noise Characterization (BLOCKER)
Before any compression decisions can be made, we need empirical noise floor data.

- Capture 50-100 static frames with camera mounted, stable lighting
- Run WHT on all 8×8 blocks of frame diffs
- Measure AC coefficient distribution (mean, σ) across all static blocks
- Set adaptive threshold at 3σ above mean
- **Without this, all compression ratio estimates are unvalidated**

### Phase 2 — Python Simulation (In Progress)
`wht_sim.py` — validates branchless WHT against reference and matrix implementations. All three implementations cross-validated and passing.

Next steps in simulation:
- [ ] `noise_profile(frames)` — characterize noise floor from static frames
- [ ] `adaptive_threshold(profile, k=3)` — compute threshold from profile
- [ ] `block_coeffs(diff, bx, by)` — extract AC coefficients for one block
- [ ] `quantize(coeffs, bits)` — quantize coefficients to N bits
- [ ] `reconstruct(ref, patches)` — inverse WHT, apply to reference
- [ ] End-to-end PSNR measurement on real motion frames

### Phase 3 — FPGA Implementation
Branchless butterfly state machine already designed (see `SDD501.md`). Maps directly to Verilog.

- [ ] `wht_butterfly.v` — single butterfly unit (1 adder, 1 subtractor)
- [ ] `wht_1d.v` — sequential 8-point 1D WHT using butterfly
- [ ] `wht_2d.v` — row-then-column 2D WHT on 8×8 block
- [ ] `coeff_threshold.v` — compare AC coefficients against threshold register
- [ ] `coeff_pack.v` — pack top-N coefficients with indices for UART TX
- [ ] Wire into existing BRAM → WHT → LoRa UART pipeline in `top.v`

### Phase 4 — Transmission Format
Minimal packet format per changed block:

| Field         | Size    | Notes                    |
|---------------|---------|--------------------------|
| Block address | 2 bytes | bx, by (0-39, 0-29)      |
| Coeff count   | 1 byte  | N coefficients to follow |
| Coeff index   | 1 byte  | position in 8×8 grid     |
| Coeff value   | 2 bytes | signed 16-bit quantized  |

Per changed block with N=4: 3 + 4×3 = 15 bytes vs 64 bytes raw = **4.3× per block**.
Combined with inter-frame sparsity (typically <10% blocks changed for wildlife): **>40× total**.

### Phase 5 — Phone Side Reconstruction
- Receive LoRa packets, unpack coefficient blocks
- Inverse WHT per block
- Apply diff to stored reference frame
- Optional: Gaussian interpolation for DC-only mode (see `SDD502.md`)

## Key Files
- `wht_sim.py` — Python simulation and validation
- `SDD501.md` — WHT mathematical description and branchless implementation
- `SDD502.md` — Phone-side Gaussian reconstruction (aggressive compression mode)
- `SDD500.md` — High-level algorithm flowchart

## Open Questions
- [ ] Noise floor threshold — **blocked on static frame dataset**
- [ ] Optimal N (coefficients per block) — depends on noise floor measurement
- [ ] Quantization bit depth — 4-bit vs 8-bit per coefficient
- [ ] DC-only Gaussian mode vs full WHT mode — tradeoff between FPS and fidelity
- [ ] Phone platform — Swift (iOS) vs Kotlin (Android) vs React Native

## Acceptance Criteria
- [ ] Static frame noise floor characterized empirically
- [ ] Adaptive threshold separates noise from motion with <5% false positives
- [ ] End-to-end Python simulation shows >30× compression on real motion frames
- [ ] PSNR > 40 dB on reconstructed motion frames
- [ ] FPGA WHT module synthesizes and produces correct coefficients
- [ ] Full pipeline transmitting over LoRa at >1 FPS for typical wildlife event
