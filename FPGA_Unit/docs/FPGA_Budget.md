## **Resource Budget (with FFT):**

| Module | LUTs | BRAM | Status |
|--------|------|------|--------|
| **256-pt Pipelined FFT** | ~2500 | 4-6 | Tight |
| **LCD RGB Controller** | ~800 | 2-4 | ✅ |
| **SPI Slave (DevKit)** | ~200 | 1 | ✅ |
| **UART (LoRa)** | ~100 | 1 | ✅ |
| **Watchdog** | ~100 | 0 | ✅ |
| **Control logic** | ~500 | 1 | ✅ |
| **Total** | ~4200 | 9-12 | ✅ **FITS!** |

## **30 FPS Pipeline Timing:**

ESP32-CAM: Capture 800x600
├─ Time: ~33ms (limited by camera)
└─ Send via SPI DMA: ~20ms

DevKit V1: ML Motion Detection
├─ TFLite inference: ~50-100ms ← BOTTLENECK!
└─ Forward to FPGA via VSPI: ~10ms

FPGA: FFT Compression
├─ 2D FFT: ~2ms ✅
├─ Coefficient extraction: ~1ms
├─ UART to LoRa: ~1:0ms (depends on baud rate)
└─ LCD update: continuous

Total latency: ~100-150ms
Actual FPS: ~7-10 FPS
```
ESP32-CAM: Capture 800x600
├─ Time: ~33ms (limited by camera)
└─ Send via SPI DMA: ~20ms

DevKit V1: ML Motion Detection
├─ TFLite inference: ~50-100ms ← BOTTLENECK!
└─ Forward to FPGA via VSPI: ~10ms

FPGA: FFT Compression
├─ 2D FFT: ~2ms ✅
├─ Coefficient extraction: ~1ms
├─ UART to LoRa: ~10ms (depends on baud rate)
└─ LCD update: continuous

Total latency: ~100-150ms
Actual FPS: ~7-10 FPS
