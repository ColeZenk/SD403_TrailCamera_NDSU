# Power Management & Switching System

## Summary
Implement a power management architecture where the ESP32 DevKitV1 acts as the sole always-on master controller, managing power to all other subsystems based on PIR motion detection. All other processors (ESP32-CAM, ESP32-S3, Tang Nano 9K FPGA) are fully power-gated when idle.

## Power States

| Subsystem       | Idle                        | Active                  |
|-----------------|-----------------------------|-------------------------|
| ESP32 DevKitV1  | Deep sleep, PIR wake only   | Full active             |
| ESP32-CAM       | Power gated off             | On after PIR trigger    |
| Tang Nano 9K    | Power gated off             | On for compression      |
| LoRa Module     | Power gated off             | On with active subsystem|
| PIR Sensors     | Always on, hardware wake    | Always on               |

## DevKitV1 Idle Behavior
The DevKitV1 is the only processor that remains powered in idle. It must consume minimal current:
- Enter deep sleep between PIR polls
- PIR GPIO configured as external wakeup source
- On PIR wake: power up subsystems in sequence, begin capture pipeline
- After capture window closes (configurable timeout), power down all subsystems and return to deep sleep

This is the primary mechanism for achieving the 25-day battery autonomy target.

## Power-On Sequence (TBD on PCB design)
Order matters to avoid inrush and ensure stable rails before dependents come up:
1. DevKitV1 wakes from deep sleep
2. Assert power switch for FPGA
3. Assert power switch for ESP32-CAM
4. 5. Wait for subsystem ready signals before starting pipeline
5. On timeout or capture complete: reverse sequence, return to deep sleep

## Power Switching
- Each subsystem controlled by a dedicated power switch (implementation TBD — discrete MOSFET or integrated load switch)
- DevKitV1 controls switches via GPIO
- Pull-down resistors on switch control lines to ensure subsystems remain off during DevKitV1 deep sleep and power-on before GPIOs are configured
- Inrush current limiting TBD on PCB design

## Power Supply
- Source: Solar panel feeding LiPo via MPPT charge controller (implementation TBD)
- Single regulated rail feeding all subsystems through their respective switches
- Rail voltage and regulator TBD based on subsystem requirements

## Firmware — DevKitV1 Changes
- [ ] Configure PIR pins as deep sleep wakeup sources (`esp_sleep_enable_ext0_wakeup` or `ext1`)
- [ ] Implement power switch GPIO control with sequenced enable/disable
- [ ] Add configurable capture window timeout
- [ ] Add `power_manager.c` module — owns all switch GPIO state
- [ ] Ensure all SPI/I2C pins are tri-stated before deep sleep to prevent backfeed through signal lines
- [ ] Log wakeup reason on boot for debugging

## Open Questions
- [ ] Power switch implementation — discrete P-channel MOSFET vs integrated load switch IC
- [ ] Whether LoRa module is switched with S3 or independently
- [ ] Inrush current budget per subsystem
- [ ] Solar panel sizing and MPPT IC selection
- [ ] Whether DevKitV1 itself runs from a separate always-on LDO or the main rail

## Acceptance Criteria
- [ ] DevKitV1 enters deep sleep and wakes reliably on PIR trigger
- [ ] All other subsystems measure near-zero current draw when power gated
- [ ] Full pipeline comes up correctly after cold power-on sequence
- [ ] System returns to deep sleep correctly after capture window
- [ ] 25-day battery autonomy target validated by current measurement
