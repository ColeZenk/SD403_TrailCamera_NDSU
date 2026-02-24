/**
 * aht20.c — AHT20 temperature/humidity sensor driver (ESP-IDF v5.1)
 */

#include "peripherals/aht20.h"
#include "peripherals/i2c_bus.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "aht20";

#define CMD_SOFT_RESET      0xBA
#define CMD_INIT            0xBE
#define CMD_TRIGGER_MEAS    0xAC
#define READ_LEN            6

static esp_err_t soft_reset(aht20_t *dev)
{
    uint8_t cmd = CMD_SOFT_RESET;
    esp_err_t err = i2c_bus_write(dev->addr, &cmd, 1, dev->timeout_ms);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(40));
    return ESP_OK;
}

static esp_err_t send_init_cmd(aht20_t *dev)
{
    uint8_t cmd[3] = { CMD_INIT, 0x08, 0x00 };
    esp_err_t err = i2c_bus_write(dev->addr, cmd, sizeof(cmd), dev->timeout_ms);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t read6(aht20_t *dev, uint8_t out[READ_LEN])
{
    return i2c_bus_read(dev->addr, out, READ_LEN, dev->timeout_ms);
}

static esp_err_t common_init_sequence(aht20_t *dev)
{
    esp_err_t err = soft_reset(dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "soft reset failed: %s (continuing)", esp_err_to_name(err));
    }

    err = send_init_cmd(dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init command failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t data[READ_LEN] = {0};
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        err = read6(dev, data);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "AHT20 ready (status 0x%02X)", data[0]);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "post-init read[%d] failed: %s", i, esp_err_to_name(err));
    }
    return err;
}

esp_err_t aht20_init(aht20_t *dev, uint8_t dev_addr)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    memset(dev, 0, sizeof(*dev));
    dev->addr       = dev_addr;
    dev->timeout_ms = 1000;

    return common_init_sequence(dev);
}

esp_err_t aht20_read_temperature_humidity(aht20_t *dev,
                                          float *temp_c,
                                          float *rh_percent)
{
    if (!dev || !temp_c || !rh_percent) return ESP_ERR_INVALID_ARG;

    uint8_t cmd[3] = { CMD_TRIGGER_MEAS, 0x33, 0x00 };
    esp_err_t err = i2c_bus_write(dev->addr, cmd, sizeof(cmd), dev->timeout_ms);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(90));

    uint8_t data[READ_LEN] = {0};
    for (int tries = 0; tries < 10; tries++) {
        err = read6(dev, data);
        if (err != ESP_OK) return err;

        if ((data[0] & 0x80) == 0) {
            uint32_t hum_raw  = ((uint32_t)data[1] << 12)
                              | ((uint32_t)data[2] << 4)
                              | (((uint32_t)data[3] >> 4) & 0x0F);

            uint32_t temp_raw = (((uint32_t)data[3] & 0x0F) << 16)
                              | ((uint32_t)data[4] << 8)
                              |  (uint32_t)data[5];

            *rh_percent = (float)hum_raw  * 100.0f / 1048576.0f;
            *temp_c     = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "sensor stayed busy");
    return ESP_ERR_TIMEOUT;
}
