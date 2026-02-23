// main/aht20.h
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint32_t timeout_ms;
} aht20_t;

// Initialize sensor handle and attempt soft reset / init sequence.
// Returns ESP_OK on success.
esp_err_t aht20_init(aht20_t *dev, i2c_master_dev_handle_t i2c_dev);

// Read temperature (°C) and relative humidity (%)
esp_err_t aht20_read_temperature_humidity(aht20_t *dev, float *temp_c, float *rh_percent);

#ifdef __cplusplus
}
#endif
