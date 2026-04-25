/**
 * @file sensors_temp_humidity.h
 * @brief Sensor subsystem — AHT20 + PIR + stepper motor
 */

#pragma once

#include "esp_err.h"

esp_err_t sensors_init(void);
void      sensors_task(void *pvParameters);
int       sensors_get_stepper_phase(void);
void      sensors_get_last_readings(float *temp_c, float *humidity_pct);
