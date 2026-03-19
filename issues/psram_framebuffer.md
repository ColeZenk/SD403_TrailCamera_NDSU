# PSRAM Frame Buffer (Replace BRAM)

## Summary
Replace the 32KB BSRAM frame buffer in the FPGA with the GW1NR-9C's embedded 64Mbit (8MB) HyperRAM PSRAM. This eliminates the need for compression just to fit a frame into the display pipeline and removes the 32KB `FPGA_MAX_TRANSFER_SIZE` constraint.

## Background
The Tang Nano 9K (GW1NR-9C) has 8MB of co-packaged PSRAM accessible via standard HyperBus I/O pins — no Gowin IP required. The current `bram_image_buffer.v` uses a 15-bit address ([14:0]) covering 32KB, which is only ~25% of a 480×272 grayscale frame (130,560 bytes). This forced a 4:1 compression requirement that wouldn't be needed with PSRAM.

With PSRAM:
- Full 480×272 frame (130,560 bytes) fits with ~60× headroom
- Double buffering is trivially possible
- `FPGA_MAX_TRANSFER_SIZE` grows to 130,560 (sent in chunks over SPI DMA)
- WHT compression pipeline can focus on LoRa bandwidth, not BRAM fit

## HyperBus Interface
Standard signals (no Gowin IP, plain Verilog):
- `CLK` — clock
- `CS#` — chip select
- `RWDS` — read/write data strobe (also signals latency during reads)
- `DQ[7:0]` — bidirectional 8-bit data

Verify exact pin assignments from the Tang Nano 9K `.cst` constraint file before starting.

## Tasks

### FPGA Side
- [ ] Identify PSRAM pin assignments in Tang Nano 9K constraints
- [ ] Write `hyperram_ctrl.v` — HyperRAM controller (burst write, burst read, configurable latency)
- [ ] Replace `bram_image_buffer.v` instantiation in `top.v` with `hyperram_ctrl.v`
- [ ] Update address bus width: `[14:0]` → `[16:0]` (17 bits covers 130,560 bytes)
- [ ] Validate timing: HyperRAM initial latency vs LCD read throughput

### ESP32 DevKitV1 Side
- [ ] Update `FPGA_MAX_TRANSFER_SIZE` in `config.h` (32768 → 130560)
- [ ] Update `fpga_spi.c` to send full frame in chunks (ESP-IDF DMA chains descriptors)
- [ ] Update `TEST_IMAGE_SIZE` constant

### Verification
- [ ] Synthesize with yosys/nextpnr — confirm timing closure at 27MHz system clock
- [ ] Confirm LCD controller reads cleanly from PSRAM during SPI write (arbitration)
- [ ] End-to-end: send test pattern from ESP32, verify display output

## Open Questions
- [ ] Read/write arbitration between SPI write port and LCD read port — need to check HyperRAM latency vs LCD pixel clock requirements
- [ ] HyperRAM initialization sequence (config registers must be written on power-up)
- [ ] Whether 27MHz system clock is fast enough to hide HyperRAM latency from LCD controller, or if a FIFO/line buffer is needed

## Key Files
- `FPGA_Unit/src/rtl/bram_image_buffer.v` — module to replace
- `FPGA_Unit/src/rtl/top.v` — instantiation site
- `ESP_Ecosystem/esp32_devkitv1/main/inc/config.h` — `FPGA_MAX_TRANSFER_SIZE`, `TEST_IMAGE_SIZE`
- `ESP_Ecosystem/esp32_devkitv1/main/src/interprocessor/fpga_spi.c` — chunked transfer logic
