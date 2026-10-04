/**
 * @file sensors_temp_humidity.h
 * @brief Sensor subsystem - AHT20 + PIR + stepper motor + DS3231 RTC
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

esp_err_t sensors_init(void);
void      sensors_task(void *pvParameters);
int       sensors_get_stepper_phase(void);
void      sensors_get_last_readings(float *temp_c, float *humidity_pct);

/** Last RTC time read by the sensor task (PIR trigger or 5 s status read).
 *  Returns false if no valid reading has been taken yet. */
bool      sensors_get_last_time(struct tm *t);

/** Live read of the DS3231 as a Unix timestamp (RTC wall-clock treated as
 *  UTC). Returns 0 if the RTC is unavailable. */
uint32_t  sensors_get_epoch(void);
