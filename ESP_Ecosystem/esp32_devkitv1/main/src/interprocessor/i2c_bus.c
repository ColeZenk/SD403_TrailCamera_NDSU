/**
 * i2c_bus.c — Shared I2C bus manager
 *
 * Mutex-protected bus access. All I2C peripherals go through here.
 */

#include "peripherals/i2c_bus.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "I2C";

#define DEFAULT_CLK  100000

static struct {
    i2c_port_t        port;
    SemaphoreHandle_t mutex;
    bool              initialized;
} bus;

esp_err_t i2c_bus_init(const i2c_bus_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (bus.initialized) return ESP_ERR_INVALID_STATE;

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = cfg->sda_gpio,
        .scl_io_num       = cfg->scl_gpio,
        .sda_pullup_en    = cfg->enable_internal_pullup ? GPIO_PULLUP_ENABLE
                                                        : GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = cfg->enable_internal_pullup ? GPIO_PULLUP_ENABLE
                                                        : GPIO_PULLUP_DISABLE,
        .master.clk_speed = cfg->clk_speed_hz ? cfg->clk_speed_hz : DEFAULT_CLK,
    };

    esp_err_t err;

    err = i2c_param_config(cfg->port, &conf);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(cfg->port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return err;

    bus.mutex = xSemaphoreCreateMutex();
    if (!bus.mutex) {
        i2c_driver_delete(cfg->port);
        return ESP_ERR_NO_MEM;
    }

    bus.port = cfg->port;
    bus.initialized = true;

    ESP_LOGI(TAG, "ready — port %d, SDA=%d SCL=%d, %lu Hz",
             cfg->port, cfg->sda_gpio, cfg->scl_gpio,
             cfg->clk_speed_hz ? cfg->clk_speed_hz : (uint32_t)DEFAULT_CLK);
    return ESP_OK;
}

i2c_port_t i2c_bus_get_port(void)        { return bus.port; }
bool       i2c_bus_is_init(void)          { return bus.initialized; }

esp_err_t i2c_bus_lock(uint32_t timeout_ms)
{
    if (!bus.initialized) return ESP_ERR_INVALID_STATE;
    return (xSemaphoreTake(bus.mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
           ? ESP_OK : ESP_ERR_TIMEOUT;
}

void i2c_bus_unlock(void)
{
    if (bus.mutex) xSemaphoreGive(bus.mutex);
}

esp_err_t i2c_bus_write(uint8_t addr, const uint8_t *data,
                        size_t len, uint32_t timeout_ms)
{
    if (!bus.initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = i2c_bus_lock(timeout_ms);
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(bus.port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);

    i2c_bus_unlock();
    return err;
}

esp_err_t i2c_bus_read(uint8_t addr, uint8_t *data,
                       size_t len, uint32_t timeout_ms)
{
    if (!bus.initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = i2c_bus_lock(timeout_ms);
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(bus.port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);

    i2c_bus_unlock();
    return err;
}

esp_err_t i2c_bus_write_read(uint8_t addr,
                             const uint8_t *wr, size_t wr_len,
                             uint8_t *rd, size_t rd_len,
                             uint32_t timeout_ms)
{
    if (!bus.initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = i2c_bus_lock(timeout_ms);
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, wr, wr_len, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (rd_len > 1) i2c_master_read(cmd, rd, rd_len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &rd[rd_len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(bus.port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);

    i2c_bus_unlock();
    return err;
}

esp_err_t i2c_bus_deinit(void)
{
    if (!bus.initialized) return ESP_OK;

    esp_err_t err = i2c_driver_delete(bus.port);
    if (err != ESP_OK) return err;

    if (bus.mutex) { vSemaphoreDelete(bus.mutex); bus.mutex = NULL; }
    bus.initialized = false;
    return ESP_OK;
}
