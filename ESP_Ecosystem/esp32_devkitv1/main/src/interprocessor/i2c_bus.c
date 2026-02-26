/**
 * i2c_bus.c — Shared I2C bus manager (ESP-IDF v5.1 legacy driver)
 */

#include "peripherals/i2c_bus.h"
#include "peripherals/motor.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";

#define DEFAULT_CLK_SPEED  100000

static struct {
    i2c_port_t        port;
    SemaphoreHandle_t mutex;
    bool              initialised;
} s_bus;

esp_err_t i2c_bus_init(const i2c_bus_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    if (s_bus.initialised) {
        ESP_LOGW(TAG, "bus already initialised");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = cfg->sda_gpio,
        .scl_io_num       = cfg->scl_gpio,
        .sda_pullup_en    = cfg->enable_internal_pullup ? GPIO_PULLUP_ENABLE
                                                        : GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = cfg->enable_internal_pullup ? GPIO_PULLUP_ENABLE
                                                        : GPIO_PULLUP_DISABLE,
        .master.clk_speed = cfg->clk_speed_hz ? cfg->clk_speed_hz
                                               : DEFAULT_CLK_SPEED,
    };

    esp_err_t err = i2c_param_config(cfg->port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(cfg->port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    s_bus.mutex = xSemaphoreCreateMutex();
    if (!s_bus.mutex) {
        i2c_driver_delete(cfg->port);
        ESP_LOGE(TAG, "failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    s_bus.port = cfg->port;
    s_bus.initialised = true;

    ESP_LOGI(TAG, "bus ready on port %d  SDA=%d SCL=%d  %lu Hz",
             cfg->port, cfg->sda_gpio, cfg->scl_gpio,
             cfg->clk_speed_hz ? cfg->clk_speed_hz : (uint32_t)DEFAULT_CLK_SPEED);
    return ESP_OK;
}

i2c_port_t i2c_bus_get_port(void)
{
    return s_bus.port;
}

esp_err_t i2c_bus_lock(uint32_t timeout_ms)
{
    if (!s_bus.initialised) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_bus.mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void i2c_bus_unlock(void)
{
    if (s_bus.mutex) xSemaphoreGive(s_bus.mutex);
}

esp_err_t i2c_bus_write(uint8_t dev_addr, const uint8_t *data,
                        size_t len, uint32_t timeout_ms)
{
    if (!s_bus.initialised) return ESP_ERR_INVALID_STATE;

    esp_err_t err = i2c_bus_lock(timeout_ms);
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(s_bus.port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);

    i2c_bus_unlock();
    return err;
}

esp_err_t i2c_bus_read(uint8_t dev_addr, uint8_t *data,
                       size_t len, uint32_t timeout_ms)
{
    if (!s_bus.initialised) return ESP_ERR_INVALID_STATE;

    esp_err_t err = i2c_bus_lock(timeout_ms);
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(s_bus.port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);

    i2c_bus_unlock();
    return err;
}

esp_err_t i2c_bus_write_read(uint8_t dev_addr,
                             const uint8_t *write_data, size_t write_len,
                             uint8_t *read_data, size_t read_len,
                             uint32_t timeout_ms)
{
    if (!s_bus.initialised) return ESP_ERR_INVALID_STATE;

    esp_err_t err = i2c_bus_lock(timeout_ms);
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_data, write_len, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (read_len > 1) {
        i2c_master_read(cmd, read_data, read_len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &read_data[read_len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(s_bus.port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);

    i2c_bus_unlock();
    return err;
}

esp_err_t i2c_bus_deinit(void)
{
    if (!s_bus.initialised) return ESP_OK;

    esp_err_t err = i2c_driver_delete(s_bus.port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_delete: %s", esp_err_to_name(err));
        return err;
    }

    if (s_bus.mutex) {
        vSemaphoreDelete(s_bus.mutex);
        s_bus.mutex = NULL;
    }

    s_bus.initialised = false;
    ESP_LOGI(TAG, "bus de-initialised");
    return ESP_OK;
}

bool i2c_bus_is_init(void)
{
    return s_bus.initialised;
}
