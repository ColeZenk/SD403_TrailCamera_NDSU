// main/aht20.c
#include "aht20.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "aht20";

/*
AHT20 basics:
- Soft reset: 0xBA
- Init:       0xBE 0x08 0x00
- Trigger:    0xAC 0x33 0x00
- Read:       6 bytes (status + 20-bit RH + 20-bit T)
*/

#define AHT20_CMD_SOFT_RESET      0xBA
#define AHT20_CMD_INIT            0xBE
#define AHT20_CMD_TRIGGER_MEAS    0xAC

#define AHT20_INIT_ARG1           0x08
#define AHT20_INIT_ARG2           0x00

#define AHT20_TRIG_ARG1           0x33
#define AHT20_TRIG_ARG2           0x00

#define AHT20_READ_LEN            6

static esp_err_t aht20_soft_reset(aht20_t *dev)
{
    uint8_t cmd = AHT20_CMD_SOFT_RESET;
    esp_err_t err = i2c_master_transmit(dev->i2c_dev, &cmd, 1, dev->timeout_ms);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(40));
    return ESP_OK;
}

static esp_err_t aht20_send_init(aht20_t *dev)
{
    uint8_t cmd[3] = { AHT20_CMD_INIT, AHT20_INIT_ARG1, AHT20_INIT_ARG2 };
    esp_err_t err = i2c_master_transmit(dev->i2c_dev, cmd, sizeof(cmd), dev->timeout_ms);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t aht20_trigger_measurement(aht20_t *dev)
{
    uint8_t cmd[3] = { AHT20_CMD_TRIGGER_MEAS, AHT20_TRIG_ARG1, AHT20_TRIG_ARG2 };
    return i2c_master_transmit(dev->i2c_dev, cmd, sizeof(cmd), dev->timeout_ms);
}

static esp_err_t aht20_read6(aht20_t *dev, uint8_t out[AHT20_READ_LEN])
{
    return i2c_master_receive(dev->i2c_dev, out, AHT20_READ_LEN, dev->timeout_ms);
}

esp_err_t aht20_init(aht20_t *dev, i2c_master_dev_handle_t i2c_dev)
{
    if (!dev || !i2c_dev) return ESP_ERR_INVALID_ARG;

    memset(dev, 0, sizeof(*dev));
    dev->i2c_dev = i2c_dev;
    dev->timeout_ms = 1000;

    // Soft reset may fail on some boards/adapters even when reads later work
    esp_err_t err = aht20_soft_reset(dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "soft reset failed: %s (continuing)", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "soft reset OK");
    }

    err = aht20_send_init(dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init command failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "init command OK");

    // Try reading status bytes a few times after init
    uint8_t data[AHT20_READ_LEN] = {0};
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        err = aht20_read6(dev, data);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "post-init read[%d]: %02X %02X %02X %02X %02X %02X",
                     i, data[0], data[1], data[2], data[3], data[4], data[5]);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "post-init read[%d] failed: %s", i, esp_err_to_name(err));
    }

    return err;
}

esp_err_t aht20_read_temperature_humidity(aht20_t *dev, float *temp_c, float *rh_percent)
{
    if (!dev || !dev->i2c_dev || !temp_c || !rh_percent) return ESP_ERR_INVALID_ARG;

    esp_err_t err = aht20_trigger_measurement(dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "trigger failed: %s", esp_err_to_name(err));
        return err;
    }

    // Typical measurement time ~80 ms
    vTaskDelay(pdMS_TO_TICKS(90));

    uint8_t data[AHT20_READ_LEN] = {0};

    // Poll BUSY bit (status bit7 = 1 means busy)
    bool ready = false;
    for (int tries = 0; tries < 10; tries++) {
        err = aht20_read6(dev, data);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read6 failed (try %d): %s", tries, esp_err_to_name(err));
            return err;
        }

        if ((data[0] & 0x80) == 0) {
            ready = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!ready) {
        ESP_LOGW(TAG, "sensor stayed busy");
        return ESP_ERR_TIMEOUT;
    }

    uint32_t hum_raw = ((uint32_t)data[1] << 12) |
                       ((uint32_t)data[2] << 4)  |
                       (((uint32_t)data[3] >> 4) & 0x0F);

    uint32_t temp_raw = (((uint32_t)data[3] & 0x0F) << 16) |
                        ((uint32_t)data[4] << 8) |
                        (uint32_t)data[5];

    *rh_percent = ((float)hum_raw * 100.0f) / 1048576.0f;   // 2^20
    *temp_c     = ((float)temp_raw * 200.0f) / 1048576.0f - 50.0f;

    return ESP_OK;
}