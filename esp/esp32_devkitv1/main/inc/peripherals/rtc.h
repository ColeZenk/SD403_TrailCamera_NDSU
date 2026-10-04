/**
 * @file rtc.h
 * @brief DS3231 RTC driver (time get/set, temp, power-loss flag)
 *
 * Uses the legacy ESP-IDF I2C master API on a port that has already been
 * initialised (i2c_bus_init). DS3231 is at 0x68; the AT24C32 EEPROM on the
 * same module (0x57) is not used.
 *
 * SETTING THE CLOCK:
 *   1. Uncomment RTC_SET_TIME_ONCE below.
 *   2. Build, flash, boot once (log shows "RTC set to ...").
 *   3. Comment it out again, rebuild, reflash.
 * Leaving it enabled resets the RTC to the stale build time on every boot.
 */
#pragma once

#include <stdbool.h>
#include <time.h>

#include "driver/i2c.h"
#include "esp_err.h"

// #define RTC_SET_TIME_ONCE

#define DS3231_I2C_ADDR 0x68

/** Bind to an initialised I2C port and verify the chip responds. */
esp_err_t ds3231_init(i2c_port_t port);

/** Read time into tm (24h, tm_year since 1900, tm_mon 0-11). */
esp_err_t ds3231_get_time(struct tm *t);

/** Write time (years 2000-2099) and clear the oscillator-stop flag. */
esp_err_t ds3231_set_time(const struct tm *t);

/** True if the oscillator stopped (battery died / first power-up). */
esp_err_t ds3231_lost_power(bool *lost);

/** Die temperature in deg C (0.25 C resolution). */
esp_err_t ds3231_get_temp(float *temp_c);

/** Copy RTC time into the ESP32 system clock (settimeofday). */
esp_err_t ds3231_sync_system_time(void);

#ifdef RTC_SET_TIME_ONCE
#warning "RTC_SET_TIME_ONCE is enabled - RTC will be overwritten at every boot"

/** Set RTC to the time this firmware was compiled (build PC local time). */
esp_err_t ds3231_set_time_from_build(void);

/** Set RTC to an explicit date/time (year e.g. 2026, month 1-12). */
esp_err_t ds3231_set_time_manual(int year, int month, int day, int hour, int min,
                              int sec);
#endif
