// peripherals/motor_i2c.c
#include "peripherals/motor_i2c.h"
#include "peripherals/i2c_bus.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "motor_i2c";

static esp_err_t write_output_reg(uint8_t val)
{
    uint8_t buf[2] = { MOTOR_OUTPUT_REG, val };
    esp_err_t err = i2c_bus_write(MOTOR_I2C_ADDR, buf, sizeof(buf), 100);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "write 0x%02X failed: %s", val, esp_err_to_name(err));
    return err;
}

esp_err_t motor_i2c_enable(void)
{
    return write_output_reg(0x00);
}

esp_err_t motor_i2c_disable(void)
{
    return write_output_reg(MOTOR_BIT_EN);
}

esp_err_t motor_i2c_move(int steps, int direction)
{
    if (steps <= 0) return ESP_ERR_INVALID_ARG;

    uint8_t dir_bit = (direction >= 0) ? MOTOR_BIT_DIR : 0;

    esp_err_t err = write_output_reg(dir_bit);
    if (err != ESP_OK) return err;

    for (int i = 0; i < steps; i++) {
        err = write_output_reg(dir_bit | MOTOR_BIT_STEP);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(MOTOR_I2C_PULSE_MS));

        err = write_output_reg(dir_bit);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(MOTOR_I2C_STEP_DELAY_MS));
    }

    return ESP_OK;
}

esp_err_t motor_i2c_swing_cw(int steps, int hold_ms)
{
    esp_err_t err = motor_i2c_move(steps, +1);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    err = motor_i2c_move(steps, -1);
    if (err != ESP_OK) return err;
    return motor_i2c_disable();
}

esp_err_t motor_i2c_swing_ccw(int steps, int hold_ms)
{
    esp_err_t err = motor_i2c_move(steps, -1);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    err = motor_i2c_move(steps, +1);
    if (err != ESP_OK) return err;
    return motor_i2c_disable();
}

esp_err_t motor_i2c_read_buttons(uint8_t *buttons)
{
    if (!buttons) return ESP_ERR_INVALID_ARG;
    uint8_t reg = MOTOR_INPUT_REG;
    esp_err_t err = i2c_bus_write_read(MOTOR_I2C_ADDR, &reg, 1, buttons, 1, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "button read failed: %s", esp_err_to_name(err));
        return err;
    }
    *buttons = (~(*buttons)) & 0x1F;
    return ESP_OK;
}