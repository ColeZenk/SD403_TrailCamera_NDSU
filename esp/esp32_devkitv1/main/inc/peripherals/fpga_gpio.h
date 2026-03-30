/**
 * fpga_gpio.h — FPGA I2C GPIO Expander Driver
 *
 * PCA9534-compatible slave at address 0x27 on the Tang Nano 9K.
 *
 * Register map (set in config.h):
 *   0x00  OUTPUT_PORT  — drive stepper coils on bits[3:0]
 *   0x01  INPUT_PORT   — read buttons on bits[2:0] (after invert: 1=pressed)
 *   0x02  DIR_REG      — 1=input, 0=output  (boot: 0xF0)
 *   0x03  INVERT_REG   — polarity invert     (boot: 0x07)
 *
 * Wiring:
 *   ESP32 GPIO_NUM_21 (SDA) ── 4.7kΩ ── 3.3V ── FPGA pin 82
 *   ESP32 GPIO_NUM_22 (SCL) ── 4.7kΩ ── 3.3V ── FPGA pin 81
 *   Common GND
 *
 * Requires i2c_bus to be initialised before calling fpga_gpio_init().
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * Initialise the GPIO expander.
 * Writes DIR_REG and INVERT_REG with boot defaults from config.h.
 *
 * @return ESP_OK on success
 */
esp_err_t fpga_gpio_init(void);

/**
 * Drive the stepper coil outputs (bits[3:0] of OUTPUT_PORT).
 *
 * @param pattern  Lower 4 bits written to OUTPUT_PORT (bit0=step_1 … bit3=step_4)
 * @return ESP_OK on success
 */
esp_err_t fpga_gpio_set_steppers(uint8_t pattern);

/**
 * Read button state from INPUT_PORT.
 * After hardware invert (INVERT_REG=0x07): bit set = button pressed.
 *
 * @param[out] buttons  Bits[2:0]: bit0=L, bit1=R, bit2=S
 * @return ESP_OK on success
 */
esp_err_t fpga_gpio_read_buttons(uint8_t *buttons);

#ifdef TEST_MODE_FPGA_GPIO
/**
 * Test task: polls buttons every 50 ms, logs state changes,
 * and mirrors button presses to stepper outputs as a simple pattern.
 *
 * Button_L pressed → step_1 only
 * Button_R pressed → step_2 only
 * Button_S pressed → all steppers on
 * No button        → all steppers off
 *
 * @param pvParameters  Unused
 */
void fpga_gpio_test_task(void *pvParameters);
#endif
