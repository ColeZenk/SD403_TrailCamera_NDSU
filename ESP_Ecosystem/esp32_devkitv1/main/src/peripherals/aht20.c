/**
 * aht20.c — AHT20 temperature/humidity sensor driver
 *
 * Uses i2c_bus.h for bus access. Init → trigger → read.
 */

#include "peripherals/aht20.h"
#include "peripherals/i2c_bus.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "AHT20";

#define CMD_RESET     0xBA
#define CMD_INIT      0xBE
#define CMD_MEASURE   0xAC
#define READ_LEN      6

static esp_err_t soft_reset(aht20_t *dev)
{
    uint8_t cmd = CMD_RESET;
    esp_err_t err = i2c_bus_write(dev->addr, &cmd, 1, dev->timeout_ms);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(40));
    return ESP_OK;
}

static esp_err_t send_init(aht20_t *dev)
{
    uint8_t cmd[3] = { CMD_INIT, 0x08, 0x00 };
    esp_err_t err = i2c_bus_write(dev->addr, cmd, sizeof(cmd), dev->timeout_ms);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

esp_err_t aht20_init(aht20_t *dev, uint8_t addr)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    memset(dev, 0, sizeof(*dev));
    dev->addr       = addr;
    dev->timeout_ms = 1000;

    /* Reset → init → wait for ready */
    soft_reset(dev);

    esp_err_t err = send_init(dev);
    if (err != ESP_OK) return err;

    uint8_t data[READ_LEN];
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        err = i2c_bus_read(dev->addr, data, READ_LEN, dev->timeout_ms);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "ready (status 0x%02X)", data[0]);
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "init failed");
    return err;
}

esp_err_t aht20_read(aht20_t *dev, float *temp_c, float *rh_pct)
{
    if (!dev || !temp_c || !rh_pct) return ESP_ERR_INVALID_ARG;

    uint8_t cmd[3] = { CMD_MEASURE, 0x33, 0x00 };
    esp_err_t err = i2c_bus_write(dev->addr, cmd, sizeof(cmd), dev->timeout_ms);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(90));

    uint8_t data[READ_LEN];
    for (int i = 0; i < 10; i++) {
        err = i2c_bus_read(dev->addr, data, READ_LEN, dev->timeout_ms);
        if (err != ESP_OK) return err;

        /* Bit 7 of status = busy */
        if ((data[0] & 0x80) == 0) {
            uint32_t hum_raw  = ((uint32_t)data[1] << 12)
                              | ((uint32_t)data[2] << 4)
                              | ((data[3] >> 4) & 0x0F);

            uint32_t temp_raw = (((uint32_t)data[3] & 0x0F) << 16)
                              | ((uint32_t)data[4] << 8)
                              |  (uint32_t)data[5];

            *rh_pct  = (float)hum_raw  * 100.0f / 1048576.0f;
            *temp_c  = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "sensor busy — timeout");
    return ESP_ERR_TIMEOUT;
}
