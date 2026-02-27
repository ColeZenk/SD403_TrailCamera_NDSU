/**
 * aht20.h — AHT20 temperature/humidity driver
 *
 * Depends on i2c_bus module. Call i2c_bus_init() before aht20_init().
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

#define AHT20_I2C_ADDR_DEFAULT  0x38

typedef struct {
    uint8_t addr;
    int     timeout_ms;
} aht20_t;

esp_err_t aht20_init(aht20_t *dev, uint8_t dev_addr);

esp_err_t aht20_read(aht20_t *dev,
                     float *temp_c,
                     float *rh_percent);
