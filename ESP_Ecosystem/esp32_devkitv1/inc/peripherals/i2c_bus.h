/**
 * i2c_bus.h — Reusable I2C bus manager for ESP-IDF v5.1 (legacy driver)
 */

#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

typedef struct {
    i2c_port_t port;
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    uint32_t   clk_speed_hz;       /**< 0 → default 100 kHz */
    bool       enable_internal_pullup;
} i2c_bus_config_t;

esp_err_t  i2c_bus_init(const i2c_bus_config_t *cfg);
i2c_port_t i2c_bus_get_port(void);
esp_err_t  i2c_bus_lock(uint32_t timeout_ms);
void       i2c_bus_unlock(void);

esp_err_t i2c_bus_write(uint8_t dev_addr, const uint8_t *data,
                        size_t len, uint32_t timeout_ms);

esp_err_t i2c_bus_read(uint8_t dev_addr, uint8_t *data,
                       size_t len, uint32_t timeout_ms);

esp_err_t i2c_bus_write_read(uint8_t dev_addr,
                             const uint8_t *write_data, size_t write_len,
                             uint8_t *read_data, size_t read_len,
                             uint32_t timeout_ms);

esp_err_t i2c_bus_deinit(void);
bool      i2c_bus_is_init(void);
