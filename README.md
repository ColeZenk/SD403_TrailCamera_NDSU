# SD403_TrailCamera_NDSU
**Work in Progress** | Senior Design Project | NDSU EE 2025-2026


## Brief

Remote trail camera system with 2-mile LoRa transmission using FPGA-accelerated WHT compression. Multi-processor architecture spanning ESP32-CAM, ESP32-DevKit, Tang Nano 9K FPGA, ESP32-S3 receiver, and mobile interface.

## Architectural Overview
```mermaid

%%{init: {'theme':'base', 'themeVariables': {'primaryTextColor':'#000','textColor':'#000'}}}%%
flowchart LR
    subgraph trailcam[Trail Cam IV]
        CAM[Camera<br/>OV2640]
        MAIN[Main Controller<br/>ESP32-DevKit]
        FPGA[FPGA<br/>Tang Nano 9K]
        LORA_TX[LoRa TX<br/>REYAX]
    end

    subgraph receiver[Receiver]
        LORA_RX[LoRa RX<br/>REYAX]
        S3[Reconstruction<br/>ESP32-S3]
        APP[Mobile App]
    end

    CAM -->|SPI DMA<br/>Image Data| MAIN
    MAIN -->|SPI DMA<br/>Image Data| FPGA
    MAIN -->|UART<br/>Packets| LORA_TX
    LORA_TX -.->|RF 915MHz<br/>2 miles| LORA_RX
    LORA_RX -->|Coefficients| S3
    S3 -->|Reconstructed<br/>BLE/WiFi| APP

    style trailcam fill:#e8f5e9,stroke:#000,stroke-width:2px
    style receiver fill:#e3f2fd,stroke:#000,stroke-width:2px
    style CAM fill:#fff,stroke:#000,color:#000
    style MAIN fill:#fff,stroke:#000,color:#000
    style FPGA fill:#fff,stroke:#000,color:#000
    style LORA_TX fill:#fff,stroke:#000,color:#000
    style LORA_RX fill:#fff,stroke:#000,color:#000
    style S3 fill:#fff,stroke:#000,color:#000
    style APP fill:#fff,stroke:#000,color:#000
```


## Components
- **ESP32-CAM** Image capture and SD storage
- **ESP32-DevKit** Sensor management and motor control
- **Tang Nano 9K FPGA** WHT compression and TFT display
- **ESP32-S3** WHT reconstruction and mobile API
- **Mobile App** Cross-platform UI (Flutter)

## Documentation
- [Software Design Documents](./Documentation/SDDs/SDD000.md)
- [Test Documents](./Documentation/TPD.md)
- [BOM Summary](./Documentation/BOM/BOM000.md)
- [Simulation](./sim/doc/SDD500.md)
## Project Info
- **Budget** <$300
- **Goal Range** 2 miles
- **Team** 3 members
  - Cole Zenk
  - Rhett Hudoba
  - Collin Lunde

## Issues
- [gpio extender](./issues/gpio_expander.md)
- [WHT compression](./issues/WHT_compression.md)
- [Power switching](./issues/power_switching.md)

## Todos and Timeline

```mermaid
---
config:
  theme: dark
---
gantt
    title Trail Camera IV — Senior Design Timeline
    dateFormat  YYYY-MM-DD
    excludes    weekends

    %% ================================
    %% FOUNDATION
    %% ================================
    section Foundation
    FPGA RTL Architecture           :done,  fpga1, 2025-11-18, 3d
    LCD Base Framework              :done,  lcd1,  after fpga1, 3d
    Camera Initialization           :done,  cam1,  after lcd1, 2d
    Documentation Setup             :done,  doc1,  after cam1, 1d
    Core Platform Stable               :milestone, core1, 2025-11-24, 0d

    %% ================================
    %% HARDWARE TRACK
    %% ================================
    section Hardware & Comms
    ESP32-Camera Integration        :done,  sub1,  2025-12-02, 2d
    DMA Stream Implementation       :done,  dma1,  after sub1, 2d
    LoRa Module Integration         :done,  lora1, after dma1, 3d
    Transmission Validation         :done,  lora2, after lora1, 2d
    Hardware Complete                  :milestone, hw1, 2025-12-16, 0d

    %% ================================
    %% WHT ENGINE
    %% ================================
    section WHT Architecture
    FPGA WHT Butterfly Engine       :active, crit, fft1, 2026-02-03, 5d
    Coefficient Extraction          :crit,   fft2, after fft1, 3d
    Motion Vector Implementation    :        motion1, after fft1, 7d
    Patch-Based Compression         :crit,   patch1, after fft2, 5d
    WHT Engine Validated               :milestone, m6, 2026-02-18, 0d

    %% ================================
    %% CODEC PIPELINE
    %% ================================
    section Codec Pipeline
    DevKit Packet Assembly          :crit,   codec1, after patch1, 5d
    Receiver Reconstruction         :crit,   codec3, after codec1, 7d
    LoRa Transmission Testing       :crit,   codec2, after codec3, 3d
    Gaussian Smoothing Filter       :        codec4, after codec3, 3d
    End-to-End Validation           :crit,   test3,  after codec2, 5d
    Codec Complete                     :milestone, m7, 2026-03-10, 0d

    %% ================================
    %% OPTIMIZATION
    %% ================================
    section Optimization
    Compression Tuning              :        opt1, after test3, 5d
    Power Budget Verification       :        opt2, after opt1, 3d
    Long-Range Testing (2 Miles)    :crit,   range1, after opt2, 5d
    Image Quality Optimization      :        opt3, after range1, 5d
    Optimization Complete              :milestone, m8, 2026-03-31, 0d

    %% ================================
    %% FINAL DELIVERY
    %% ================================
    section Final Integration
    PCB Integration                    :crit,   pcb1, 2026-04-01, 7d
    Mechanical Assembly                :        mech1, 2026-04-01, 7d
    Full System Testing             :crit,   test4, after pcb1, 5d
    Bug Fixes & Refinement          :        bug1,  after test4, 5d
    Buffer / Contingency            :        buffer1, after bug1, 5d
    Demo Preparation                :crit,   demo1, 2026-04-28, 5d
    Senior Design Demonstration     :milestone, demo2, 2026-05-09, 0d
    Final Project Delivery          :milestone, m9, 2026-05-16, 0d
```
