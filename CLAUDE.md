# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Project Overview

Trail Camera IV — an embedded wildlife camera system.

**Data flow:** ESP32-CAM → (SPI slave) → ESP32 DevKitV1 → (SPI master) → Tang Nano 9K FPGA → 480×272 RGB LCD

Three independently buildable subsystems: FPGA RTL (`FPGA_Unit/`), ESP32 DevKitV1 firmware (`ESP_Ecosystem/esp32_devkitv1/`), and a Python WHT compression simulation (`sim/`).

---

## Build Commands

### FPGA (Tang Nano 9K)

```bash
cd FPGA_Unit
make              # Synthesize + PnR + pack bitstream → build/trail_camera.fs
make program      # Build and flash over USB (openFPGALoader)
make flash        # Reprogram from existing bitstream (skip rebuild)
make flash-persist  # Write to external SPI flash (survives power cycle)
make clean        # Remove build/ directory
```

**Toolchain:** `yosys` (synthesis) → `nextpnr-himbaechel` (PnR) → `gowin_pack` (bitstream) → `openFPGALoader` (programming)

### ESP32 DevKitV1 (ESP-IDF)

```bash
cd ESP_Ecosystem/esp32_devkitv1
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
idf.py -p /dev/ttyUSB0 flash monitor   # flash then open serial monitor
```

### Simulation (Python)

```bash
cd sim
source venv/bin/activate
python wht_sim.py               # Walsh-Hadamard Transform compression test
python wht_sim.py --report      # Generate PDF test report
```

---

## FPGA RTL Architecture (`FPGA_Unit/src/rtl/`)

### Coding Rules — strictly enforced

- **Branchless datapath:** Use AND-mask / OR-combine / XOR for all mux logic. `if` statements are **only** permitted for async reset (`if (!rst_n)`). All other control flow must be combinational masks.
- **3-stage synchronizers** on every async input crossing clock domains (SPI SCLK/CS, I2C SCL/SDA, button inputs).
- The yosys/nextpnr toolchain requires these patterns — deviating causes synthesis errors or metastability.

### Module hierarchy

| File | Role |
|---|---|
| `top.v` | Top-level: instantiates all submodules, power-on reset (256-cycle counter), BRAM write control, debug LED assignments |
| `esp_interface.v` | SPI Mode 0 slave — 3-stage synchronizers on SCLK/CS/MOSI, shift register with AND-masked byte output |
| `bram_image_buffer.v` | 32 KB dual-port BSRAM (write from SPI, read from LCD controller simultaneously) |
| `lcd_controller.v` | 480×272 timing generator, 13.5 MHz pixel clock (27 MHz / 2), reads grayscale from BRAM and expands to R[4:0] G[5:0] B[4:0] |
| `i2c_gpio_expander.v` | PCA9534-compatible I2C slave at address 0x27 — full state machine, open-drain SDA tristate |
| `constraints/pins.cst` | Physical pin assignments (Gowin CST format) |

### Key constraints

- **BRAM is exactly 32 768 bytes** — a single SPI CS transaction transfers the full frame. CS must not deassert mid-transfer or `bram_write_addr` resets to 0.
- Raw 480×272 grayscale = 130 560 bytes, which does not fit in BRAM. The sim/ WHT pipeline targets ~4:1 compression to get one sub-frame into BRAM.
- System clock: 27 MHz (pin 52). I2C and button pins are LVCMOS18 (3.3 V tolerant but mapped to 1.8 V bank — verify voltage domain on any new peripherals).

### Debug LEDs (active-low on Tang Nano 9K)

| `led[n]` | Signal |
|---|---|
| 0 | `receiving` (SPI CS active) |
| 1 | `frame_ready` (full frame received) |
| 2 | `esp_cs_active` |
| 3 | `esp_rx_valid` |
| 4 | Any stepper coil active |
| 5 | Any button pressed |

---

## ESP32 DevKitV1 Firmware Architecture

All constants (pins, clocks, sizes, priorities) live in **`main/inc/config.h`** — never hardcode values in `.c` files.

### Source layout

```
main/
  inc/
    config.h                  ← single source of truth for all constants
    fpga_spi.h / cam_spi.h / lora_uart.h / image_processor.h
    peripherals/fpga_gpio.h   ← I2C GPIO expander driver header
  src/
    main.c                    ← init sequence + task creation + heap monitor
    interprocessor/
      fpga_spi.c              ← SPI master to FPGA (async queue/get DMA)
      cam_spi.c               ← SPI slave from ESP32-CAM
      lora_uart.c             ← LoRa radio (UART2)
      i2c_bus.c               ← shared I2C bus init
    peripherals/
      fpga_gpio.c             ← PCA9534 I2C driver for FPGA GPIO expander
      aht20.c                 ← AHT20 temperature/humidity sensor
      motor.c                 ← stepper motor driver
      pir.c                   ← PIR motion sensor
      sensors_temp_humidity.c ← sensor task wrapper
    utilities/
      image_processor.c       ← image queue, compression pipeline
      isr.c                   ← ISR handlers + global semaphores
      utils.c                 ← dma_malloc(), min_size(), etc.
```

### Task architecture

| Task | Priority | Stack | Function |
|---|---|---|---|
| `cam_rx` | HIGH (5) | MEDIUM | Receives image frames from ESP32-CAM over SPI |
| `img_proc` | MEDIUM (4) | LARGE | Processes + compresses frames, forwards to FPGA |
| `lora_rx` | MEDIUM (4) | MEDIUM | Polls LoRa radio for commands |
| `sensors` | LOW (3) | MEDIUM | Reads AHT20 temp/humidity periodically |
| `fpga_test` | MEDIUM (4) | MEDIUM | Only when `TEST_MODE_FPGA_PATTERNS` defined |
| `fpga_gpio_test` | LOW (3) | SMALL | Only when `TEST_MODE_FPGA_GPIO` defined |

### SPI transfer pattern (`fpga_spi.c`)

Uses async queue/get to keep DMA pipeline full: queues up to `FPGA_SPI_QUEUE_SIZE` transactions, then drains results one at a time. CS stays asserted for the full 32 768-byte frame (single transaction). Any error drains remaining in-flight transactions before returning.

### I2C GPIO expander (`fpga_gpio.c`)

The FPGA implements a PCA9534-compatible I2C slave at `0x27`. On boot, the ESP32 writes `DIR_REG = 0xF0` (bits[3:0] = outputs for steppers) and `INVERT_REG = 0x07` (invert buttons so 1 = pressed). `i2c_bus_init()` must run before `fpga_gpio_init()`.

### Test modes

Controlled by `#define` in `main/inc/config.h`. Only one functional test mode should be active at a time (enforced by `#error`). Current state: `TEST_MODE_FPGA_PATTERNS` and `TEST_MODE_FPGA_GPIO` are both defined (they are independent, the `#error` guard only checks `PATTERNS + LORA_LOOPBACK` together).

| Define | Effect |
|---|---|
| `TEST_MODE_FPGA_PATTERNS` | Button on GPIO0 cycles test patterns (white/black/gradient/checker) to FPGA |
| `TEST_MODE_LORA_LOOPBACK` | LoRa echo test |
| `TEST_MODE_CAMERA_INJECT` | Inject synthetic frames into image pipeline |
| `TEST_MODE_FPGA_GPIO` | Poll buttons via I2C, mirror to steppers |

---

## Simulation (`sim/`)

`wht_sim.py` implements the Walsh-Hadamard Transform compression pipeline in Python as a reference/validation for eventual HDL implementation. The sim uses a Python venv (`sim/venv/`) with numpy and scipy. Input: raw 320×240 grayscale frames (`img_NNNN.raw`). The pipeline: box LPF → FWHT → quantization → top-K truncation → adaptive mode selection (patch vs DC+Gaussian).
