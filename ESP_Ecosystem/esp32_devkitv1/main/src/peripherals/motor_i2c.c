// peripherals/motor_i2c.c
#include "peripherals/motor_i2c.h"
#include "peripherals/i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "motor_i2c";

#define REG_OUTPUT   0x00
#define REG_INPUT    0x01
#define REG_DIR      0x02
#define REG_INVERT   0x03

// DIR_REG: bits[3:0]=outputs (steppers), bits[7:4]=inputs (buttons)
// 0=output, 1=input -> 0xF0
#define DIR_CONFIG   0xF0

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    esp_err_t err = i2c_bus_write(MOTOR_I2C_ADDR, buf, sizeof(buf), 100);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "write reg=0x%02X val=0x%02X failed: %s",
                 reg, val, esp_err_to_name(err));
    return err;
}

static esp_err_t write_output_reg(uint8_t val)
{
    return write_reg(REG_OUTPUT, val);
}

esp_err_t motor_i2c_init(void)
{
    esp_err_t err = write_reg(REG_DIR, DIR_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DIR_REG config failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "GPIO expander configured at 0x%02X, DIR=0x%02X",
             MOTOR_I2C_ADDR, DIR_CONFIG);
    return ESP_OK;
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
    uint8_t reg = REG_INPUT;
    esp_err_t err = i2c_bus_write_read(MOTOR_I2C_ADDR, &reg, 1, buttons, 1, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "button read failed: %s", esp_err_to_name(err));
        return err;
    }
    // buttons on gpio_in[4:0], active low — invert and mask 5 bits
    *buttons = (~(*buttons)) & 0x1F;
    return ESP_OK;
}