// peripherals/motor_i2c.h
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_I2C_ADDR       0x27
#define MOTOR_OUTPUT_REG     0x00
#define MOTOR_INPUT_REG      0x01

#define MOTOR_BIT_STEP       (1 << 0)
#define MOTOR_BIT_DIR        (1 << 1)
#define MOTOR_BIT_EN         (1 << 2)

#define MOTOR_I2C_STEP_DELAY_MS   2
#define MOTOR_I2C_PULSE_MS        1

esp_err_t motor_i2c_enable(void);
esp_err_t motor_i2c_disable(void);
esp_err_t motor_i2c_move(int steps, int direction);
esp_err_t motor_i2c_swing_cw(int steps, int hold_ms);
esp_err_t motor_i2c_swing_ccw(int steps, int hold_ms);
esp_err_t motor_i2c_read_buttons(uint8_t *buttons);

#ifdef __cplusplus
}
#endif