# SD403_TrailCamera_NDSU

---

## WIP

---

# Scope/Context:

SD 403/405 is the final demonstration of applied skills for soon to be graduating NDSU students.
The project chosen for this group(6) was to create a trail cam with the ability to capture images with an ESP32-CAM and transmit it to a phone 2 miles away including relavand data (sensor info). An additional requirement of a TFT UI was also included in the initial BOR.
This group decided to up the anti so to speak, and devised a structure that could transmit *live* data 2 miles away.

---

## Plan:

Use FFTs to deconstruct and reconstruct images as well as sensor data.
 **This feature will implement an FPGA in the design (GOWIN Tang Nano 9K)**

## Processor responsibilities:

### ESP32-CAM:
 - Capture, store, and pass images rapidly to ESP32-DEVKITV1

### ESP32-DEVKITV1:
  - Serve as intermediary between CAM and FPGA.
  - House (back end) edge ML model for motion prediction.
  - Manage stepper motor, RTC, and temperature/humidity sensor.
  - Manage pre-rendered backgrounds for TFT UI.

### Tang Nano 9K:

  - Manage TFT display interfacing.
  - Compress (pre-process) image streams to be transmitted via LoRa(UART).
  - House ML data lake (micro sd) for ESP32-DEVKITV1.
  - Manage low power watch dog system with PIR sensors.
  - Serve as a "smart IC" for extra pin needs (going to be connected in parallel bit transfer so this is needed)

### ESP32-S3 (WROOM):
  - Recieve FFT coefficients.
  - House first daisy ML reconstruction mechanism.
  - Functionally serve as an API for iPhone.

### iPhone:
  - Final virtual user interface.
  - Polish and patch partitioned ML reconstructed image data.
  - On/Off, motor control, training en/dis.

---

# Table Of Contents

## Build Target Docs
- [ESP32-Cam](./ESP_Ecosystem/esp32cam/docs/index.md)
- [ESP32-DevkitV1](./ESP_Ecosystem/esp32_devkitv1/docs/index.md)
- [ESP32-S3 (WROOM)](./ESP_Ecosystem/esp32s3_wroom/docs/index.md)
- [Tang Nano 9k](./FPGA_Unit/docs/Index.md)

## Physical Features and Budgets
- [Total Power Budget]()
- [Pin Allocations]()
-


---

## Programming standards:

*Also included in project OneNote under: /"Project Charter"/"Relevant Standards"*

1. Modular file system. I.e. multiple concise (ideally less that 300 lines) files and corresponding header files. Not one monolithic main file per microcontroller.

2. Document pin allocations, feature implementations and so on either by updating the relevant readme.md, commit message, or both. Headers on files for clarity.

3. Try to commit often.

4. Try to adhere to UNIX philosophy if relevant.

5. Adhere to C standards Verilog standards, and dart(flutter) standards when applicable.
