# SD403_TrailCamera_NDSU
**Work in Progress** | Senior Design Project | NDSU EE 2025-2026


## Brief

Remote trail camera system with 2-mile LoRa transmission using FPGA-accelerated FFT compression. Multi-processor architecture spanning ESP32-CAM, ESP32-DevKit, Tang Nano 9K FPGA, ESP32-S3 receiver, and mobile interface.

## Architectural Overview
TODO

## Components
- **ESP32-CAM** Image capture and SD storage
- **ESP32-DevKit** Sensor management and motor control
- **Tang Nano 9K FPGA** FFT compression and TFT display
- **ESP32-S3** FFT reconstruction and mobile API
- **Mobile App** Cross-platform UI (Flutter)

## Documentation
- [Software Design Documents](./Documentation/SDDs/SDD000.md)
- [Test Documents](./Documentation/TPD.md)
- [BOM Summary](./Documentation/BOM/BOM000.md)

## Project Info
- **Budget** <$300
- **Goal Range** 2 miles
- **Team** 3 members
  - Cole Zenk
  - Rhett Hudoba
  - Collin Lunde

## Todos and Timline

```mermaid
%%{init: {'theme':'forest'}}%%
gantt
    dateFormat  YYYY-MM-DD
    title       Trail Camera IV - Senior Design Project Timeline
    excludes    weekends

    section Phase 1: Foundation (Nov 2025)
    FPGA RTL Structure           :done,    fpga1, 2025-11-18, 3d
    LCD Foundation               :done,    lcd1, 2025-11-18, 3d
    Camera Initialization        :done,    cam1, 2025-11-20, 2d
    README Documentation         :done,    doc1, 2025-11-22, 1d

    section Phase 2: Camera System (Dec 2025)
    ESP32-Camera Submodule       :done,    sub1, 2025-12-02, 2d
    Camera Capture Working       :done,    cam2, 2025-12-04, 3d
    File Separation              :done,    file1, 2025-12-09, 2d
    DMA Stream Implementation    :done,    dma1, 2025-12-11, 2d
    Unit Testing (Camera)        :done,    test1, 2025-12-11, 1d
    Camera Activation Complete   :milestone, m1, 2025-12-13, 0d

    section Phase 3: LoRa Communication (Dec 2025)
    DevKit UART Setup            :done,    uart1, 2025-12-09, 2d
    LoRa Module Integration      :done,    lora1, 2025-12-11, 3d
    LoRa Working                 :done,    lora2, 2025-12-14, 2d
    LoRa Comm Complete           :milestone, m2, 2025-12-16, 0d

    section Phase 4: LCD & DMA (Dec 2025)
    Test Bench Implementation    :done,    test2, 2025-12-16, 2d
    LCD Controller               :done,    lcd2, 2025-12-18, 3d
    SPI DMA Integration          :done,    dma2, 2025-12-20, 2d
    LCD Backstrapping Complete   :milestone, m3, 2025-12-23, 0d

    section Phase 5: Integration (Jan 2026)
    Systems Integration PR       :done,    int1, 2026-01-13, 3d
    Senior Design Task 4         :done,    task4, 2026-01-16, 2d
    Integration Testing          :done,    int2, 2026-01-20, 2d
    System Integration Complete  :milestone, m4, 2026-01-24, 0d

    section Phase 6: Documentation (Jan 2026)
    Documentation Overhaul       :done,    doc2, 2026-01-26, 2d
    README Updates               :done,    doc3, 2026-01-30, 1d
    SDD Documentation            :done,    doc4, 2026-01-30, 1d
    Typo Fixes                   :done,    doc5, 2026-01-31, 1d
    Documentation Complete       :milestone, m5, 2026-01-31, 0d

    section Phase 7: FFT Implementation (Feb 2026)
    FPGA FFT IP Integration      :active,  fft1, 2026-02-03, 5d
    Coefficient Extraction       :         fft2, after fft1, 3d
    Motion Vector Implementation :         motion1, 2026-02-03, 7d
    Patch-based Compression      :         patch1, after fft2, 5d
    FFT Module Complete          :milestone, m6, 2026-02-18, 0d

    section Phase 8: Codec Integration (Feb-Mar 2026)
    DevKit Packet Assembly       :         codec1, after fft2, 5d
    LoRa Transmission Testing    :         codec2, after codec1, 3d
    Receiver Reconstruction      :         codec3, after codec2, 7d
    Gaussian Smoothing           :         codec4, after codec3, 3d
    End-to-End Testing           :crit,    test3, after codec4, 5d
    Codec Working                :milestone, m7, 2026-03-10, 0d

    section Phase 9: Optimization (Mar 2026)
    Compression Tuning           :         opt1, after test3, 5d
    Power Budget Validation      :         opt2, after opt1, 3d
    Range Testing (2 miles)      :crit,    range1, after opt2, 5d
    Image Quality Optimization   :         opt3, after range1, 5d
    System Optimization Complete :milestone, m8, 2026-03-31, 0d

    section Phase 10: Final Integration (Apr 2026)
    PCB Integration              :crit,    pcb1, 2026-04-01, 7d
    Mechanical Assembly          :         mech1, 2026-04-01, 7d
    Full System Testing          :crit,    test4, after pcb1, 5d
    Bug Fixes & Refinement       :         bug1, after test4, 5d

    section Phase 11: Demo & Delivery (May 2026)
    Demo Preparation             :crit,    demo1, 2026-04-28, 5d
    Final Documentation          :         doc6, 2026-04-28, 7d
    Senior Design Demo           :milestone, demo2, 2026-05-09, 0d
    Project Delivery             :milestone, m9, 2026-05-16, 0d
```
