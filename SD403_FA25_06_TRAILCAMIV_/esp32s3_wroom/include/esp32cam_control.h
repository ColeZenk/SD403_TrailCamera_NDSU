/*
 * ESP32-CAM Control Header
 * ESP32-S3 WROOM Master - Boot mode and reset control for ESP32-CAM
 */

#ifndef ESP32CAM_CONTROL_H
#define ESP32CAM_CONTROL_H

#include "driver/gpio.h"

// Control Pin Definitions
#define CAM_IO0_PIN         GPIO_NUM_1      // Controls IO0 on ESP32-CAM (boot mode)
#define CAM_RESET_PIN       GPIO_NUM_2      // Controls EN/Reset on ESP32-CAM

/**
 * Initialize control pins for ESP32-CAM
 * Sets up GPIO pins for boot mode and reset control
 */
void esp32cam_control_init(void);

/**
 * Enter programming/bootloader mode on ESP32-CAM
 * Sequence: IO0 LOW -> Reset pulse -> Ready for programming
 */
void esp32cam_enter_programming_mode(void);

/**
 * Reset ESP32-CAM to normal boot mode
 * Sequence: IO0 HIGH -> Reset pulse -> Normal operation
 */
void esp32cam_reset_to_normal_mode(void);

/**
 * Perform hard reset on ESP32-CAM
 * Does not change boot mode, just resets the device
 */
void esp32cam_hard_reset(void);

/**
 * Set boot mode without resetting
 * @param bootloader_mode true for bootloader mode, false for normal mode
 */
void esp32cam_set_boot_mode(bool bootloader_mode);

#endif // ESP32CAM_CONTROL_H